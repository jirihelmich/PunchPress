#include <rtems.h>
#include <assert.h>
#include <bsp.h>
#include <rtems/irq-extension.h>

/**
 * out ports
 */
#define OUT_PWR_X 0x7070
#define OUT_PWR_Y 0x7071
#define OUT_PUNCH_IRQ 0x7072

/**
 * bitwise arithmetic macros
 */
#define HEAD_IS_IN_SAFE_ZONE_LEFT(VAL)   (VAL & 1 << (4))
#define HEAD_IS_IN_SAFE_ZONE_TOP(VAL)    (VAL & 1 << (6))
#define SAFE_B(VAL)  				     (VAL & 1 << (7))
#define SAFE_R(VAL)                      (VAL & 1 << (5))
#define ENC_X_VAL(VAL)                   (VAL & 0b0011)
#define ENC_Y_VAL(VAL)                   ((VAL & 0b1100) >> 2)

#define AXIS_X 0
#define AXIS_Y 1

/**
 * Tasks periods [ms]
 */
#define TASK1_PERIOD 10
#define TASK2_PERIOD 50

/**
 * max speed used for calibration
 */
#define CALIBRATION_POWER		-60

/**
 * movement directions constants
 */
#define MOVEMENT_LEFT_TOP       -1
#define MOVEMENT_NOWHERE         0
#define MOVEMENT_RIGHT_BOTTOM    1

/**
 * how many cycles should pass to make sure we are stayingtr
 */
#define CALIBRATION_CALM_COUNTER_LIMIT      100

/**
 * magic constants for motor speed computation
 */
#define MOTOR_SPEED_CONSTANT_POSITION       0.25*1.15
#define MOTOR_SPEED_CONSTANT_DELTA          -1.05

/**
 * motor threshold + max power
 */
#define MAXIMAL_MOTOR_POWER 127
#define MINIMAL_MOTOR_POWER 18

/**
 * position data structure
 */
struct position{
	int x;
	int y;
};
typedef struct position postion_t;

/**
 * semaphore guarding the critical sections of code (status R/W)
 */
volatile static rtems_id state_semaphore_id;

/**
 * current position of the head. this is set by the interrupt handler.
 */
volatile static postion_t pos;

/**
 * punchpress status list
 */
#define STATE_INITIAL        0
#define STATE_READY          1
#define STATE_NAVIGATING     2
#define STATE_PUNCH_READY    3
#define STATE_PUNCHING       4
#define STATE_RETRACT        5
#define STATE_DONE           6

/**
 * punchpress current state
 */
static int PUNCHPRESS_STATE = STATE_INITIAL;

/**
 * Where the head should go. This is used by both the planning task and the speed-controlling
 * task. But the mutual exclusion is made by the punchpress states. If the task is planning,
 * the second one is not navigating.
 */
volatile static postion_t destination;

/**
 * Old delta for purposes of the speed-controlling task only.
 */
static postion_t old_delta;

/**
 * where to punch the holes
 */
static int holes[][2] = {
		{30,20},
		{120, 30},
		{110, 80},
		{40, 100}
};

static rtems_id task1_id;
static rtems_id task2_id;

// this is needed for calibration purpose - to remember the previous value of the encoder
uint8_t OLD_ENC_X = -1;
uint8_t OLD_ENC_Y = -1;

/**
 * Decides in which direction the head moves
 */
int get_direction(int oldval, int currval)
{
	if (oldval == currval) return 0;

	//10, 11, 01, 00
	if ((oldval == 0b10) && (currval == 0b11)) return 1;
	if ((oldval == 0b11) && (currval == 0b01)) return 1;
	if ((oldval == 0b01) && (currval == 0b00)) return 1;
	if ((oldval == 0b00) && (currval == 0b10)) return 1;

	return -1;
}

/**
 * interrupt handler
 */
void isr(void *dummy) {

	uint32_t input;
	i386_inport_byte(0x7070, input); //read position encoders

	uint8_t enc_x = ENC_X_VAL(input);
	uint8_t enc_y = ENC_Y_VAL(input);

	pos.x -= get_direction(OLD_ENC_X, enc_x);
	pos.y -= get_direction(OLD_ENC_Y, enc_y);

	OLD_ENC_X = enc_x;
	OLD_ENC_Y = enc_y;
}

/**
 * Signum function as described on SOF
 */
static inline int signum (float num)
{
	return (0 < num) - (num < 0);
}

/**
 * http://www.go4expert.com/forums/showthread.php?t=8714
 */
static inline int abs (int x)
{
	int t = x >> 31;
	// t is -1 if x is negative otherwise it is 0

	return t ^ (x + t);
}

/**
 * makes sure that the value is between -MAXIMAL_MOTOR_POWER and MAXIMAL_MOTOR_POWER
 *
 * if the value is not equal to 0, it also makes sure that the absolute value of the
 * input is greater than MINIMAL_MOTOR_POWER
 *
 * if any rule is not satisfied, the return value is altered to satisfy the rules
 */
float saturate(float num)
{
	int sgn = signum(num);

	// get absolute value of the float
	float newval = (sgn == -1 ? -num : num);

	// value is greater than maximal motor power
	if (newval > MAXIMAL_MOTOR_POWER)
	{
		newval = MAXIMAL_MOTOR_POWER; // normalize
	}else if (newval < MINIMAL_MOTOR_POWER) // threshold check
	{
		if (sgn != 0) // means the value is in the interval (0,MAXIMAL_MOTOR_POWER), but not 0 -> increase
		{
			newval = MINIMAL_MOTOR_POWER;
		}
	}

	// apply sign again
	if (sgn == -1)
	{
		newval = -newval;
	}

	return newval;
}

/**
 * tries to determine the best speed at the moment
 */
float compute_motor_speed(int delta, int old_delta)
{
	/**
	 * E = G-P
	 * F = E*Kp+(E-E')*Kd
	 */
	float f = ((delta+1)*MOTOR_SPEED_CONSTANT_POSITION + abs(delta-old_delta)*MOTOR_SPEED_CONSTANT_DELTA);
	f = saturate(f);
	return f;
}

/**
 * aquire semaphore, set new status, release the semaphore
 */
void set_status(int status)
{
	rtems_status_code status_sem = rtems_semaphore_obtain(state_semaphore_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	if ( status_sem != RTEMS_SUCCESSFUL )
	{
		return;
	}

	/**
	 * CRITICAL SECTION
	 */
	PUNCHPRESS_STATE = status;
	/**
	 * END OF CRITICAL SECTION
	 */

	rtems_semaphore_release(state_semaphore_id);
}

/**
 * Returns the duration of the period in count of the ticks during the period.
 */
static inline rtems_interval get_ticks_for_period(int period)
{
	int factor = 1000/period;
	return rtems_clock_get_ticks_per_second()/factor;
}

/**
 * Reads punchpress state safely while using semaphore
 */
static inline int read_punchpress_state()
{
	// aquire semaphore
	rtems_status_code status = rtems_semaphore_obtain(state_semaphore_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	if (status != RTEMS_SUCCESSFUL)
	{
		return -1;
	}

	/**
	 * CRITICAL SECTION
	 */
	int state = PUNCHPRESS_STATE;
	/**
	 * CRITICAL SECTION END
	 */

	// release the semaphore
	rtems_semaphore_release(state_semaphore_id);

	return state;
}

/**
 * Task which controls the speed of the head based on current location
 * and based on where the head has to go.
 */
rtems_task Task1(rtems_task_argument ignored) {
	rtems_status_code status;
	rtems_id          period_id;
	rtems_interval    ticks;

	status = rtems_rate_monotonic_create(
			rtems_build_name( 'P', 'E', 'R', '1' ),
			&period_id
	);
	ticks = get_ticks_for_period(TASK1_PERIOD);

	/**
	 * How far away from the target we are?
	 */
	postion_t delta;
	int standing_cycles_count = 0;

	while(1)
	{
		status = rtems_rate_monotonic_period( period_id, ticks );
		if(status == RTEMS_TIMEOUT)
		{
			break; // this is the end. the system missed a deadline, which is fatal.
		}

		int state = read_punchpress_state();

		// error while aquiring the semaphore
		if (state < STATE_INITIAL) break;

		// we are capable of navigation only. anything else is ignored
		if (state != STATE_NAVIGATING) continue;

		// compute current delta
		delta.x = destination.x - pos.x;
		delta.y = destination.y - pos.y;

		// compute new motors speed
		int newx = compute_motor_speed(delta.x, old_delta.x);
		int newy = compute_motor_speed(delta.y, old_delta.y);

		// propagate new power
		outport_byte(OUT_PWR_X, newx);
		outport_byte(OUT_PWR_Y, newy);

		// increase if standing, reset if moving away
		standing_cycles_count = (standing_cycles_count+1)*((newx == 0) && (newy == 0));

		// standing for long enaugh
		if (standing_cycles_count > CALIBRATION_CALM_COUNTER_LIMIT)
		{
			// get ready to punch!
			standing_cycles_count = 0;
			set_status(STATE_PUNCH_READY);
		}

		// remember current position
		old_delta.x = delta.x;
		old_delta.y = delta.y;
	}

	printf("ERROR! DEADLINE MISSED IN TASK CONTROLLING SPEED!\n");
}

/**
 * decides where to move next. if there is still a hole to punch, moves over there.
 * if it does not, moves to [0;0].
 *
 * after doing the math, sets the status to STATE_NAVIGATING to let the other task
 * to move the head
 */
static void plan_movement(int hole_to_punch, int total_holes_count)
{
	if (total_holes_count <= hole_to_punch)
	{
		// nothing to punch, just dock
		destination.x = 0;
		destination.y = 0;
	}else
	{
		destination.x = holes[hole_to_punch][0]*4;
		destination.y = holes[hole_to_punch][1]*4;
	}

	set_status(STATE_NAVIGATING);
}

/**
 * perform the actual punching. if the number of the hole is out of range,
 * we've just docked. so set the status to DONE.
 *
 * otherwise, set signal to punch (don't disable interrupts!) and hold
 * for 2ms
 */
void punch(int hole_to_punch, int holes_total_count)
{
	if (hole_to_punch >= holes_total_count)
	{
		// docked, set status to DONE
		set_status(STATE_DONE);
		return;
	}

	// set the punch signal and hold for 2ms
	outport_byte(OUT_PUNCH_IRQ, 0b11);
	rtems_task_wake_after(get_ticks_for_period(2));

	// set status PUNCHING (aka the head should be down)
	set_status(STATE_PUNCHING);
}

/**
 * The head should be down or moves to this state. "wait" until
 * the HEAD_UP signal is false. don't literally wait, just
 * see if the head is down. If it is, begin retracting and set
 * RETRACT state.
 */
void control_punch(int * hole_to_punch)
{
	uint32_t input;
	inport_byte(0x7071, input);

	if ((input & 1) == 0){ // is head down?
		outport_byte(OUT_PUNCH_IRQ, 0b10); // begin retract
		set_status(STATE_RETRACT); // status "head is moving up"
		*hole_to_punch += 1; // loop to the next hole
	}
}

/**
 * The head is moving up. "Wait" until the head is really up.
 * Then set status READY to signal we are ready for next hole.
 */
void control_retract()
{
	uint32_t input;
	inport_byte(0x7071, input);
	if ((input & 1) == 1) // is head already UP?
	{
		set_status(STATE_READY);
	}
}

/**
 *
 */
void docked()
{
	rtems_task_delete(task1_id);
	rtems_task_delete(task2_id);

}

/**
 *
 */
rtems_task Task2(rtems_task_argument ignored) {
	rtems_status_code status;
	rtems_id          period_id;
	rtems_interval    ticks;

	status = rtems_rate_monotonic_create(
			rtems_build_name( 'P', 'E', 'R', '2' ),
			&period_id
	);
	ticks = get_ticks_for_period(50);

	// the punching will start with the first hole
	int hole_to_punch = 0;

	// we are about to punch holes_total_count holes
	int holes_total_count = 4;

	while(1)
	{
		status = rtems_rate_monotonic_period( period_id, ticks );
		if(status == RTEMS_TIMEOUT)
		{
			break; // this is the end. the system missed a deadline, which is fatal.
		}

		int error = 0;
		int state = read_punchpress_state();
		if (state < STATE_INITIAL) break;

		switch (state)
		{
		case STATE_READY:
			plan_movement(hole_to_punch, holes_total_count);
			break;
		case STATE_PUNCH_READY:
			punch(hole_to_punch, holes_total_count);
			break;
		case STATE_PUNCHING:
			control_punch(&hole_to_punch);
			break;
		case STATE_RETRACT:
			control_retract();
			break;
		case STATE_NAVIGATING:
			break;
		case STATE_DONE:
			docked();
			break;
		default:
			error = 1;
			break;
		}

		if (error){
			break;
		}
	}

	printf("ERROR! SOMETHING WENT WRONG (UNEXPECTED STATE OR DEADLINE MISSED) IN TASK CONTROLLING PUNCHING!\n");

}

/**
 * Determines if the head has moved since the last check.
 * If it has not, the not_moved parameter value is increased by one,
 * if it has, the value is set to 0.
 */
void is_moving(int *oldval, int currval, int *not_moved)
{
    if((*oldval != currval)){
        *not_moved = 0;
    }else{
        (*not_moved)++;
    }
    *oldval = currval;
}

/**
 * Function used to decide whether the head is already in safe zone. If it is, stop the motor and store the current position.
 * If it is not, continue moving.
 */
void calibration_logic_axis(int axis, uint32_t input, int *was_in_safezone, int *safezone_enter, int *oldpos, int *not_moved)
{
	int in_safezone = (axis == AXIS_X ? HEAD_IS_IN_SAFE_ZONE_LEFT(input) : HEAD_IS_IN_SAFE_ZONE_TOP(input));
	int pwr = (axis == AXIS_X ? OUT_PWR_X : OUT_PWR_Y);
	int currval = (axis == AXIS_X ? pos.x : pos.y);

    if(in_safezone == 0)
	{
		i386_outport_byte(pwr, CALIBRATION_POWER);
	}else{
		if (*was_in_safezone == 0)
		{
			*safezone_enter = currval;
		}

		i386_outport_byte(pwr, 0);
		is_moving(oldpos, currval, not_moved);
		*was_in_safezone = 1;
	}
}

/**
 * Calibrate the head to obtain a better location info than "in the middle of nowhere".
 * Goes to the top-left corner of the work area. While reaching the safe zone, it remembers
 * the current position and counts the real-world position.
 *
 * Since the power to the motor on the appropriate axis is cut off when crossing the safe zone,
 * the motor will eventually stop. Checks, that the motor stays still for N iterations.
 * After that, the head really stays still, so no interrupt is triggered, so it is safe
 * to update the position data structure.
 */
void calibrate()
{

	int x_not_moved = 0;
	int y_not_moved = 0;

	int oldpos_x = 0;
	int oldpos_y = 0;

	int safezone_enter_x = -1;
	int safezone_enter_y = -1;

	int was_in_safezone_x = 0;
	int was_in_safezone_y = 0;

	// enables interrupt (set 2nd bit of port 0x7072 to true)
	i386_outport_byte(0x7072, 0b10);

	uint32_t input = 0;
	while (1){
		i386_inport_byte(0x7070, input);

		// set power or detect safe zone for each of the axis
		calibration_logic_axis(AXIS_X, input, &was_in_safezone_x, &safezone_enter_x, &oldpos_x, &x_not_moved);
		calibration_logic_axis(AXIS_Y, input, &was_in_safezone_y, &safezone_enter_y, &oldpos_y, &y_not_moved);

		if ((x_not_moved > CALIBRATION_CALM_COUNTER_LIMIT) && (y_not_moved > CALIBRATION_CALM_COUNTER_LIMIT))
		{
			break;
		}
	}

	pos.x -= safezone_enter_x;
	pos.y -= safezone_enter_y;

	old_delta.x = 0;
	old_delta.y = 0;
}

/**
 *
 */
void create_and_start_tasks()
{
	// calibration done, setup new tasks and start them
	rtems_status_code status;
	rtems_name task_name = rtems_build_name( 'T', 'A', '1', ' ' );

	status = rtems_task_create(
			task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
			RTEMS_DEFAULT_ATTRIBUTES, &task1_id
	);
	assert( status == RTEMS_SUCCESSFUL );

	status = rtems_task_start( task1_id, Task1, 1 );
	assert( status == RTEMS_SUCCESSFUL );


	/**************************************************************/

	task_name = rtems_build_name( 'T', 'A', '2', ' ' );

	status = rtems_task_create(
			task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
			RTEMS_DEFAULT_ATTRIBUTES, &task2_id
	);
	assert( status == RTEMS_SUCCESSFUL );

	status = rtems_task_start( task2_id, Task2, 1 );
	assert( status == RTEMS_SUCCESSFUL );
}

/**
 * Initial task.
 *
 * Enables interrupts, runs calibration and starts up new tasks.
 *
 * Delete itself after completion.
 */
rtems_task Init(rtems_task_argument ignored) {

	/*
	 * We do not know, where we are, but we need to initialize the value to get correct behavior.
	 */
	pos.x = 0;
	pos.y = 0;

	// setup IRQ handler
	rtems_status_code status;
	status = rtems_interrupt_handler_install(5, NULL, RTEMS_INTERRUPT_UNIQUE, isr, NULL);
	assert( status == RTEMS_SUCCESSFUL );

	// calibrate
	calibrate();

	// create semaphore
	rtems_name name = rtems_build_name('S','E','M','1');
	status = rtems_semaphore_create(name,1,RTEMS_SIMPLE_BINARY_SEMAPHORE,0,&state_semaphore_id);
	assert(status == RTEMS_SUCCESSFUL);

	set_status(STATE_READY);
	create_and_start_tasks();

	// all done. delete itself.
	rtems_task_delete(RTEMS_SELF);
}

/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS             3
#define CONFIGURE_MAXIMUM_PERIODS           2

#define CONFIGURE_EXTRA_TASK_STACKS (3 * RTEMS_MINIMUM_STACK_SIZE)

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT
#include <rtems/confdefs.h>


/****************  END OF CONFIGURATION INFORMATION  ****************/
