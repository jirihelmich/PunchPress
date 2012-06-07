#include <rtems.h>
#include <assert.h>
#include <bsp.h>
#include <rtems/irq-extension.h>

#define PWR_X 0x7070
#define PWR_Y 0x7071
#define PUNCH 0x7072

#define HEAD_IS_IN_SAFE_ZONE_LEFT(VAL)  (VAL & 1 << (4))
#define HEAD_IS_IN_SAFE_ZONE_TOP(VAL)  (VAL & 1 << (6))
#define SAFE_B(VAL)  (VAL & 1 << (7))
#define SAFE_R(VAL)  (VAL & 1 << (5))

#define ENC_X_VAL(VAL)  (VAL & 0b0011)
#define ENC_Y_VAL(VAL)  ((VAL & 0b1100) >> 2)

#define AXIS_X 0
#define AXIS_Y 1

#define CALIBRATION_POWER		-60

#define MOVEMENT_LEFT_TOP       -1
#define MOVEMENT_NOWHERE         0
#define MOVEMENT_RIGHT_BOTTOM    1
#define CALIBRATION_CALM_COUNTER_LIMIT 500

#define MOTOR_SPEED_CONSTANT_POSITION 0.25*1.15
#define MOTOR_SPEED_CONSTANT_DELTA -1.05

static const int MAXIMAL_MOTOR_POWER = 127;
static const int MINIMAL_MOTOR_POWER = 18;
struct position{
	int x;
	int y;
};
typedef struct position postion_t;

/**
 * current positoin of the head. this is set by the interrupt handler.
 */
volatile static postion_t pos;

/**
 * Where the head should go. This is used by both the planning task and the speed-controlling
 * task.
 */
volatile static postion_t destination;

/**
 * Old delta for purposes of the speed-controlling task only.
 */
static postion_t old_delta;

// this is needed for calibration purpose - to remember the previous value of the encoder
uint8_t OLD_ENC_X = -1;
uint8_t OLD_ENC_Y = -1;

/**
 * Decides in which direction the head moves
 */
int move_left_bottom(int oldval, int currval)
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
	i386_inport_byte(0x7070, input);

	uint8_t enc_x = ENC_X_VAL(input);
	uint8_t enc_y = ENC_Y_VAL(input);

	pos.x -= move_left_bottom(OLD_ENC_X, enc_x);
	pos.y -= move_left_bottom(OLD_ENC_Y, enc_y);

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

	float newval = (sgn == -1 ? -num : num);

	if (newval > MAXIMAL_MOTOR_POWER)
	{
		newval = MAXIMAL_MOTOR_POWER;
	}else if (newval < MINIMAL_MOTOR_POWER)
	{
		if (sgn != 0)
		{
			newval = MINIMAL_MOTOR_POWER;
		}
	}

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

	ticks = rtems_clock_get_ticks_per_second() / 100; // 1000 ms in a second, 1000/100 = 10 -> perform each 10 ms

	/**
	 * How far away from the target we are?
	 */
	postion_t delta;

	while(1)
	{
		status = rtems_rate_monotonic_period( period_id, ticks );
		if(status == RTEMS_TIMEOUT)
		{
			break; // this is the end. the system missed a deadline, which is fatal.
		}

		delta.x = destination.x - pos.x;
		delta.y = destination.y - pos.y;

		int newx = compute_motor_speed(delta.x, old_delta.x);
		int newy = compute_motor_speed(delta.y, old_delta.y);

		outport_byte(PWR_X, newx);
		outport_byte(PWR_Y, newy);

		//printf("Since delta is [%d, %d] and olddelta was [%d, %d], setting speed [%d, %d]. \n", delta.x, delta.y, old_delta.x, old_delta.y, newx, newy);

		old_delta.x = delta.x;
		old_delta.y = delta.y;
	}

	printf("ERROR! DEADLINE MISSED IN TASK CONTROLLING SPEED!\n");
}

rtems_task Task2(rtems_task_argument ignored) {
	rtems_status_code status;
	rtems_id          period_id;
	rtems_interval    ticks;

	status = rtems_rate_monotonic_create(
			rtems_build_name( 'P', 'E', 'R', '2' ),
			&period_id
	);

	ticks = rtems_clock_get_ticks_per_second() / 20; // 1000 ms in a second, 1000/20 = 50 -> perform each 50 ms

	while(1)
	{
		status = rtems_rate_monotonic_period( period_id, ticks );
		if(status == RTEMS_TIMEOUT)
		{
			break; // this is the end. the system missed a deadline, which is fatal.
		}
	}

	printf("ERROR! DEADLINE MISSED IN TASK CONTROLLING SPEED!\n");

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
	int pwr = (axis == AXIS_X ? PWR_X : PWR_Y);
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
	rtems_id task_id;
	rtems_name task_name = rtems_build_name( 'T', 'A', '1', ' ' );

	status = rtems_task_create(
			task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
			RTEMS_DEFAULT_ATTRIBUTES, &task_id
	);
	assert( status == RTEMS_SUCCESSFUL );

	status = rtems_task_start( task_id, Task1, 1 );
	assert( status == RTEMS_SUCCESSFUL );


	/**************************************************************/

	task_name = rtems_build_name( 'T', 'A', '2', ' ' );

	status = rtems_task_create(
			task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
			RTEMS_DEFAULT_ATTRIBUTES, &task_id
	);
	assert( status == RTEMS_SUCCESSFUL );

	status = rtems_task_start( task_id, Task2, 1 );
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
	create_and_start_tasks();

	destination.x = 200/0.25;
	destination.y = 300/0.25;

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
