#include <rtems.h>
#include <assert.h>
#include <bsp.h>

#define PWR_X 0x7070
#define PWR_Y 0x7071
#define PUNCH 0x7072

#define HEAD_IS_IN_SAFE_ZONE_LEFT(VAL)  (VAL & 1 << (4))
#define SAFE_T(VAL)  (VAL & 1 << (6))
#define SAFE_B(VAL)  (VAL & 1 << (7))
#define SAFE_R(VAL)  (VAL & 1 << (5))

#define ENC_X_VAL(VAL)  (VAL & 0b0011)
#define ENC_Y_VAL(VAL)  ((VAL & 0b1100) >> 2)

#define ENC_X_A(VAL)  (VAL & 1 << (0))
#define ENC_X_B(VAL)  (VAL & 1 << (1))
#define ENC_Y_A(VAL)  (VAL & 1 << (2))
#define ENC_Y_B(VAL)  (VAL & 1 << (3))

#define AXIS_X 0
#define AXIS_Y 1

#define CALIBRATION_POWER		-34

#define MOVEMENT_LEFT_TOP       -1
#define MOVEMENT_NOWHERE         0
#define MOVEMENT_RIGHT_BOTTOM    1

static const int CALIBRATION_CALM_COUNTER_LIMIT = 1000;
struct position{
	int x;
	int y;
};

rtems_task Task1(rtems_task_argument ignored) {
	rtems_status_code status;
	rtems_id          period_id;
	rtems_interval    ticks;

	status = rtems_rate_monotonic_create(
			rtems_build_name( 'P', 'E', 'R', '1' ),
			&period_id
	);

	ticks = rtems_clock_get_ticks_per_second() / 500;

	status = rtems_rate_monotonic_period( period_id, ticks );
}

int move_right_bottom(int oldval, int currval)
{
	//10, 11, 01, 00

	if ((oldval == 0b10) && (currval == 0b11))
	{
		return 1;
	}

	if ((oldval == 0b11) && (currval == 0b01))
	{
		return 1;
	}

	if ((oldval == 0b01) && (currval == 0b00))
	{
		return 1;
	}

	if ((oldval == 0b00) && (currval == 0b10))
	{
		return 1;
	}

	return 0;
}

int move_left_top(int oldval, int currval)
{
	return move_right_bottom(currval, oldval);
}

int moves_direction(int oldval, int currval)
{
	if (oldval == -1) return MOVEMENT_NOWHERE;
	if (oldval == currval) return MOVEMENT_NOWHERE;

	if (move_right_bottom(oldval, currval)){
		return MOVEMENT_RIGHT_BOTTOM;
	}

	if (move_left_top(oldval, currval)){
		return MOVEMENT_LEFT_TOP;
	}

	return MOVEMENT_NOWHERE;
}

int notice_position_change(int axistype, int oldval, int currval, struct position * p)
{
	int change = 0;
	if (axistype == AXIS_X){
		change = moves_direction(oldval, currval);
		p->x -= change;
	}else if (axistype == AXIS_Y){
		change = moves_direction(oldval, currval);
		p->y -= change;
	}

	return (change != MOVEMENT_NOWHERE);
}

void notice_calibration_movement_in_safe_zone(uint32_t axis, uint32_t *input, int *oldval, struct position *pos, int *not_moved)
{
    i386_outport_byte((axis == AXIS_X ? PWR_X : PWR_Y), 0);
    int currval = (axis == AXIS_X ? ENC_X_VAL(*input) : ENC_Y_VAL(*input));
    int moved = notice_position_change(axis, *oldval, currval, pos);
    if(moved){
        not_moved = 0;
    }else{
        not_moved++;
    }
    *oldval = currval;
}

rtems_task Init(rtems_task_argument ignored) {
	//rtems_status_code status;
	//rtems_id task_id;
	//rtems_name task_name = rtems_build_name( 'T', 'A', '1', ' ' );

	uint32_t input = 0;

	struct position pos;
	pos.x = -1;
	pos.y = -1;

	int oldxval = -1;
	int oldyval = -1;

	int x_not_moved = 0;
	int y_not_moved = 0;

	while (1){
		i386_inport_byte(0x7070, input);

		if(HEAD_IS_IN_SAFE_ZONE_LEFT(input) == 0)
		{
			i386_outport_byte(PWR_X, CALIBRATION_POWER);
		}else{
			notice_calibration_movement_in_safe_zone(AXIS_X, &input, &oldxval, &pos, &x_not_moved);
		}

		if(SAFE_T(input) == 0)
		{
			i386_outport_byte(PWR_Y, CALIBRATION_POWER);
		}else{

			notice_calibration_movement_in_safe_zone(AXIS_Y, &input, &oldyval, &pos, &y_not_moved);
		}

		if ((x_not_moved > CALIBRATION_CALM_COUNTER_LIMIT) && (y_not_moved > CALIBRATION_CALM_COUNTER_LIMIT))
		{
			break;
		}
	}

	printf("%f :: %f\n", pos.x*0.25, pos.y*0.25);

	/*status = rtems_task_create(
			task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
			RTEMS_DEFAULT_ATTRIBUTES, &task_id
	);
	assert( status == RTEMS_SUCCESSFUL );

	status = rtems_task_start( task_id, Task1, 1 );
	assert( status == RTEMS_SUCCESSFUL );

	rtems_task_delete(RTEMS_SELF);*/
}

/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS             2
#define CONFIGURE_MAXIMUM_PERIODS           2

#define CONFIGURE_EXTRA_TASK_STACKS (3 * RTEMS_MINIMUM_STACK_SIZE)

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT
#include <rtems/confdefs.h>


/****************  END OF CONFIGURATION INFORMATION  ****************/
