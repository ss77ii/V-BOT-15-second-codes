#include "../include/main.h"
#include "GarglonConfiguration.hpp"

#define PRE 2
#define CATCH 1
#define RELEASE 0

#define ON 1
#define OFF -1

// go straight parameters

//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 26, 6000, 3, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.35, 0, 24, 6000, 2, hardwareParameter);
//	goStraightCmPID_lib(122, 90, 127, MOVE_FORWARD, 5, 0, 3, 0.3, 0, 28, 6000, 1, hardwareParameter);

// goStraightCmPID_lib(60.0, 90, 127, MOVE_BACKWARD, 5, 0, 3, 0.40, 0, 1, 2000, 2, hardwareParameter);

// left is positive, right is negative

using namespace pros;

Point curPos;
double distance;
double angle;

void claw_control(int mode)
{
	if (mode == CATCH)
		front_piston.set_value(true);
	else if (mode == RELEASE)
		front_piston.set_value(false);

	delay(590);
}

void hook_control(int mode)
{
	if (mode == CATCH)
		back_piston.set_value(true);
	else if (mode == RELEASE)
		back_piston.set_value(false);
	delay(590);
}

void intake(int mode, int speed)
{
	if (mode == ON)
		intake_motor.move(-speed);

	else if (mode == OFF)
		intake_motor.move(0);
}

/***********************************************************************************************

AUTONOMOUS PROGRAMS

***********************************************************************************************/

void arm_moving_holding_fn(void *param)
{
	long start_time = pros::millis();

	arm_action_lib = ARM_HOLDING_POSITION;
	arm_motor.move(-80);
	delay(200);
	while (true)
	{
		if (fabs(arm_motor.get_actual_velocity()) < 3)
			break;

		if (pros::millis() - start_time > 500)
			break;

		delay(10);
	}

	arm_motor.tare_position();
	arm_motor.move_absolute(10, 127);
	delay(50);
	arm_motor.tare_position();

	int pass_target_count = 0;
	double pre_arm_target_angle = arm_move_target_angle_lib;
	double cur_arm_degree = arm_motor.get_position();
	double pre_arm_degree = cur_arm_degree;
	double Kp = 3, Ki = 0.005, Kd = 0;
	double error_kp = 0;
	double pre_error_kp = 0;
	double error_sum = 0;
	double error_change_rate = 0;
	double total_correction = 0;

	while (true)
	{
		if (pre_arm_target_angle != arm_move_target_angle_lib)
		{
			pre_arm_target_angle = arm_move_target_angle_lib;
			error_sum = 0;
			error_kp = arm_move_target_angle_lib - cur_arm_degree;
			pre_error_kp = error_kp;
			pass_target_count = 0;
		}

		cur_arm_degree = arm_motor.get_position();

		if (pass_target_count == 0)
		{
			if (cur_arm_degree == arm_move_target_angle_lib ||
				(cur_arm_degree > arm_move_target_angle_lib && pre_arm_degree < arm_move_target_angle_lib) ||
				(cur_arm_degree < arm_move_target_angle_lib && pre_arm_degree > arm_move_target_angle_lib))
			{
				pass_target_count++;
				//				pros::lcd::print(1, "AAAA=%.1f", arm_move_target_angle_lib);
			}
			else
			{
				if (cur_arm_degree < arm_move_target_angle_lib)
					arm_motor.move(abs(arm_move_speed_lib));

				else if (cur_arm_degree > arm_move_target_angle_lib)
					arm_motor.move(0 - abs(arm_move_speed_lib));
			}
		}
		else
		{
			error_kp = arm_move_target_angle_lib - cur_arm_degree;
			error_sum = error_sum + error_kp;
			error_change_rate = error_kp - pre_error_kp;
			total_correction = error_kp * Kp + error_sum * Ki + error_change_rate * Kd;
			arm_motor.move((int)total_correction);
			pre_error_kp = error_kp;
		}
		pre_arm_degree = cur_arm_degree;
		//		pros::lcd::print(3, "arm=%.1f", arm_motor.get_position());
		delay(5);
	}
}

double get_distance_back_vision(Vision vision_sensor,
								int goal_color_signature,
								long samplingNumber,
								long sensingTimeInterval_Millis,
								int min_width,
								long timeoutMillis)
{
	double distance = 0;
	double width1;
	pros::vision_object_s_t closest_goal;
	double sum = 0;
	long i = 0;
	long startTime = pros::millis();

	// 	for(i = 0; i < samplingNumber;){
	// 			closest_goal= vision_sensor.get_by_sig(0, goal_color_signature);
	// 			if(closest_goal.width < 60){
	// 				sum = sum + (double)(closest_goal.width);
	// 				i++;
	// 			}
	// 			pros::lcd::print(0, "Width=%d,  M=%d", closest_goal.width, closest_goal.x_middle_coord);
	// 			delay(sensingTimeInterval_Millis);
	// 		}
	// 	pros::lcd::print(1, "avg=%.1f", sum/((double)i));
	// waitForTouch();

	while (i < samplingNumber)
	{
		if (pros::millis() - startTime >= timeoutMillis)
			break;

		closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
		width1 = (double)(closest_goal.width);
		if (width1 >= min_width)
		{
			sum = sum + width1;
			i++;
		}
		delay(sensingTimeInterval_Millis);
	}
	if (i == 0)
		return -1;

	width1 = sum / ((double)i);
	//	pros::lcd::print(1, "Width=%.1f", width1);
	//	distance = 0 - (0.00002*width1*width1*width1) + (0.014*width1*width1) - (2.9598*width1) + 242.31;
	distance = 217.84 * powf(2.71828, -0.012 * width1);

	return distance;
}

double get_distance_front_vision(Vision vision_sensor, int goal_color_signature,
								 long samplingNumber, long sensingTimeInterval_Millis,
								 int min_width, long timeoutMillis)
{
	double distance = 0;
	double width1;
	pros::vision_object_s_t closest_goal;
	double sum = 0;
	long i = 0;
	long startTime = pros::millis();

	while (i < samplingNumber)
	{
		if (pros::millis() - startTime >= timeoutMillis)
		{
			break;
		}
		closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
		width1 = (double)(closest_goal.width);
		if (width1 >= min_width)
		{
			sum = sum + width1;
			i++;
		}
		delay(sensingTimeInterval_Millis);
	}
	if (i == 0)
	{
		return -1;
	}

	width1 = sum / ((double)i);
	distance = 224.17 * powf(2.71828, -0.012 * width1);
	//  distance = -2 * 0.00005 * width1 * width1 * width1 + 0.0125 * width1 * width1
	//	           - 2.7984 * width1 + 242.41;
	return distance;
}

void auton_60s_skills_bridge_version()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 75;
	long start_time = pros::millis();
	int intake_speed = 110;
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	pros::Task arm_holding_task(arm_moving_holding_fn, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm_holding_task");

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	delay(30);

	// //	bridge code
	//    sys_initial_robot_heading = 135;
	//
	// clawAction_1 = {0, true, 1};
	// hookAction_1 = {0, true, 1};
	// delay(100);
	// turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	// waitForTouch();
	// 	clawAction_1 = {0, true, 1};
	// 	delay(claw_action_delay_time);
	// 	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	// 	delay(300);
	// 	goStraightCmPID_lib(20, 185, 127, MOVE_FORWARD, 8, 0, 0, 0.2, 0, 5, 800, 1, hardwareParameter);
	// 	delay(100);
	// 	goStraightCmPID_lib(35, 183, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
	// 	delay(100);
	// //	turnDegreesPID_lib(185, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 900, 2, hardwareParameter);
	// //	goStraightCmPID_lib(30, 185, 127, MOVE_FORWARD, 3, 0, 0, 1, 0, 5, 1000, 1, hardwareParameter);
	// //waitForTouch();
	// 	armAction_1 = {127, 0, 20, 1};
	// 	delay(800);
	// 	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// 	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//
	// 	goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);
	// 	balance_bridge_PID_lib(127, -15, 15, 0, 0, 5000, 1, hardwareParameter);
	// 	goStraightCmPID_lib(3.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);
	//
	// 	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	// 	waitForTouch();

	// ------------------------------------------------------------------------------------ //
	//// part 1 take first blue goal
	goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	intakeAction_1 = {100, 500, 0, 100, 1};
	goStraightCmPID_lib(90, 73, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter); // 100,80,127
	new_goStraightCm_Front_Vision(25, 73, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.5, 0, 15, 0.25, 0, 1, 600, 1, hardwareParameter);

	angle = get_robot_heading_lib(hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	goStraightCmPID_lib(20, angle, 60, MOVE_FORWARD, 1.5, 0, 1.5, 0.8, 0, 0, 600, 1, hardwareParameter);

	///////catch first yellow goal
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 5, RELEASE_BRIDGE - 50, 1};
	delay(500);

	///////carry first yellow goal to bridge
	angle = 59;
	goStraightCmPID_lib(150, angle, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1550, 1, hardwareParameter); // 1650
	armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(100);

	goStraightCmPID_lib(15, angle, 127, MOVE_FORWARD, 2.5, 0, 1.5, 0.5, 0, 10, 400, 1, hardwareParameter); // 100,80,127
	///////place the first yellow goal to the bridge
	armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 50);

	angle = 70;
	goStraightCmPID_lib(5, angle, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 10, 0, 5, 300, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(100);
	goStraightCmPID_lib(70, 50, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 4, 1200, 1, hardwareParameter);

	////take first blue goal by front claw
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(27, 50, 100, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 700, 1, hardwareParameter);
	turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	goStraightCmPID_lib(27, 230, 60, MOVE_FORWARD, 3.5, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);
	delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, 350, 1};
	delay(100);

	goStraightCmPID_lib(65, 5, 127, MOVE_BACKWARD, 1.0, 0, 1.2, 0.7, 0, 10, 1000, 1, hardwareParameter); // 1500
	double distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 120);
	if (distance > 70 || distance < 10)
	{
		distance = 20;
	}
	goStraightCm_Back_Vision(distance + 20, 350, 65, DETECT_BLUE_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 800, 1, hardwareParameter);

	////take first red goal by back claw
	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE + 300, 1};
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

	goStraightCmPID_lib(112, 340, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 5, 1500, 1, hardwareParameter);

	goStraightCmPID_lib(20, 10, 70, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); // 1300
	goStraightCmPID_lib(25, 60, 80, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); // 1300
	goStraightCmPID_lib(43, 85, 80, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 1100, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	delay(100);
	goStraightCmPID_lib(15, 95, 100, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(3, 95, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 10, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
	delay(100);
	goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);

	armAction_1 = {127, 200, 0, 1};
	turnDegreesPID_lib(245, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

	////take middle high yellow goal
	delay(100); //
	new_goStraightCm_Front_Vision(20, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 800, 1, hardwareParameter);
	//	intakeAction_1 = {-10, 0, 0, -10, 1};
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(32, angle, 45, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 1000, 1, hardwareParameter);
	//		delay(100);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	//			armAction_1 = {127, 0, PRESS_BRIDGE + 170, 1};
	armAction_1 = {127, 0, 70, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1600, 2, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
	delay(400);
	angle = 95;
	goStraightCmPID_lib(90, angle, 85, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1500, 1, hardwareParameter);
	// move high yellow goal to red bridge
	armAction_1 = {127, 0, PRESS_BRIDGE - 20, 1};
	delay(300);
	goStraightCmPID_lib(15, angle, 80, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 700, 1, hardwareParameter);
	//			armAction_1 = {127, 0, PRESS_BRIDGE - 20, 1};
	delay(500);
	//			turnDegreesPID_lib(80, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 700, 2, hardwareParameter);
	////place high yellow mobile goal
	clawAction_1 = {0, false, 1};
	armAction_1 = {127, 0, PRESS_BRIDGE - 30, 1};
	delay(claw_action_delay_time + 100);
	goStraightCmPID_lib(5, angle, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 4, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);

	intakeAction_1 = {0, 0, 0, 0, 1};
	goStraightCmPID_lib(60, 90, 127, MOVE_BACKWARD, 3, 0, 1, 0.5, 0, 2, 1200, 1, hardwareParameter);
	goStraightCmPID_lib(100, 135, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 2, 1300, 1, hardwareParameter);
	goStraightCmPID_lib(110, 145, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 2, 1400, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(25, 145, 80, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 600, 1, hardwareParameter);

	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	goStraightCmPID_lib(68, 130, 127, MOVE_FORWARD, 1.5, 0, 1, 0.5, 0, 7, 1100, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 700, 2, hardwareParameter);

	intakeAction_1 = {0, 0, 0, 0, 1};
	distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 120);
	if (distance > 70 || distance < 10)
	{
		distance = 50;
	}
	goStraightCm_Back_Vision(distance + 20, 180, 70, DETECT_RED_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 1, 1000, 1, hardwareParameter);
	// take corner blue goal
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

	armAction_1 = {127, 0, 0, 1};
	goStraightCmPID_lib(50, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 800, 1, hardwareParameter); // 900
	turnDegreesPID_lib(90, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);	   // 900

	goStraightCm_Front_Vision(30, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 1.5, 0, 1.5, 0.8, 0, 0, 800, 1, hardwareParameter);
	// take last yellow goal
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
	goStraightCmPID_lib(36, 120, 80, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 800, 1, hardwareParameter);	  // 2000
	goStraightCmPID_lib(115, 130, 100, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 1500, 1, hardwareParameter); // 2000

	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	goStraightCmPID_lib(15, 110, 127, MOVE_FORWARD, 3.5, 0, 1, 0.4, 0, 0, 500, 1, hardwareParameter);
	// drop last yellow goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 125, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	hookAction_1 = {200, false, 1};
	goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 0.35, 0, 5, 1000, 1, hardwareParameter);

	// hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	// delay(hook_action_delay_time);
	goStraightCmPID_lib(30, 120, 80, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 800, 2, hardwareParameter);
	turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	goStraightCmPID_lib(35, 300, 60, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 900, 1, hardwareParameter);

	// place red goal on bridge
	delay(100);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 260, 1};
	turnDegreesPID_lib(110, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1300, 2, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 260, 1};
	goStraightCmPID_lib(70, 110, 80, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 2, 1400, 1, hardwareParameter);

	// release last blue goal to the bridge
	// armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	// delay(100);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 150);

	armAction_1 = {127, 500, 0, 1};
	goStraightCmPID_lib(70, 180, 127, MOVE_BACKWARD, 3.5, 0, 1, 1, 0, 2, 1400, 1, hardwareParameter);
	turnDegreesPID_lib(120, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);

	distance = get_distance_front_vision(front_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 120);
	if (distance > 70 || distance < 10)
	{
		distance = 50;
	}
	new_goStraightCm_Front_Vision(15, 120, 70, DETECT_BLUE_GOAL_SIG, front_vision,
								  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	goStraightCmPID_lib(distance - 14, angle, 50, MOVE_FORWARD, 1, 0, 1, 0.3, 0, 1, 1200, 1, hardwareParameter);
	// catch the goal beside the bridge
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, 60, 1};
	goStraightCmPID_lib(200, 83, 127, MOVE_BACKWARD, 2, 0, 1, 0.3, 0, 5, 2300, 1, hardwareParameter);
	goStraightCmPID_lib(40, 135, 127, MOVE_FORWARD, 1.5, 0, 1, 0.3, 0, 5, 800, 1, hardwareParameter);
	goStraightCmPID_lib(60, 135, 127, MOVE_BACKWARD, 2, 0, 1, 0.3, 0, 5, 900, 1, hardwareParameter);

	// turnDegreesPID_lib(30, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	// goStraightCmPID_lib(40, 60, 127, MOVE_FORWARD,	1, 0, 1, 0.3, 0, 1, 900, 1, hardwareParameter);
	// turnDegreesPID_lib(135, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	// goStraightCmPID_lib(40, 135, 100, MOVE_BACKWARD,	1, 0, 1, 0.3, 0, 5, 1000, 1, hardwareParameter);
	delay(100);
	distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 120);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	goStraightCm_Back_Vision(distance + 5, 135, 100, DETECT_BLUE_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 5, 1200, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	// delay(hook_action_delay_time);

	// //	bridge code

	//		goStraightCmPID_lib(40, 195, 127, MOVE_FORWARD, 8, 0, 0, 0.3, 0, 5, 1200, 1, hardwareParameter);
	goStraightCmPID_lib(5, 135, 127, MOVE_FORWARD, 2, 0, 0, 1, 0, 0, 500, 1, hardwareParameter);
	turnDegreesPID_lib(215, ON_SPOT_TURN, 60, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	//		delay(100);
	goStraightCmPID_lib(45, 215, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1000, 1, hardwareParameter);
	//		goStraightCmPID_lib(35, 180, 127, MOVE_FORWARD, 1.2, 0, 0, 0.5, 0, 5, 1200, 1, hardwareParameter);
	//		delay(100);
	armAction_1 = {127, 200, 20, 1};
	turnDegreesPID_lib(180, ON_SPOT_TURN, 60, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);

	//		armAction_1 = {127, 0, 20, 1};
	delay(200);
	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);
	balance_bridge_PID_lib(127, -15, 15, 0, 0, 5000, 1, hardwareParameter);
	goStraightCmPID_lib(3.5, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);

	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	waitForTouch();

	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////
	//	armAction_1 = {127, 0, 100, 1};
	///////////////////////////////////////?????
	//	goStraightCmPID_lib(15, 30, 100, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 0, 600, 1, hardwareParameter);

	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////

	// bridge code
	//  clawAction_1 = {0, true, 1};
	//  delay(claw_action_delay_time);
	//  armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	//  goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	//  hookAction_1 = {0, true, 1};
	//  delay(hook_action_delay_time);
	//  delay(1000);
	//  goStraightCmPID_lib(35, 180, 127, MOVE_FORWARD, 3, 0, 0, 1, 0, 5, 1000, 1, hardwareParameter);
	//  armAction_1 = {127, 0, 20, 1};
	//  delay(1000);
	//  //waitForTouch();
	//  left_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  left_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  right_front_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  right_back_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//  right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	//
	//  goStraightCmPID_lib(85, 180, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);
	//  balance_bridge_PID_lib(127, -15, 15, 0, 0, 5000, 1, hardwareParameter);
	//  goStraightCmPID_lib(3, 180, 127, MOVE_BACKWARD, 0, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);
	//

	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	waitForTouch();
}

void auton_60s_skills()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 75;
	long start_time = pros::millis();

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};

	// ------------------------------------------------------------------------------------ //
	//// part 1 take first blue goal
	goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	//	delay(200);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);

	intakeAction_1 = {90, 400, 0, 90, 1};
	goStraightCmPID_lib(90, 80, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter); // 100,80,127

	// goStraightCm_Front_Vision(25, 80, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
	//  	                  0.5, 0, 1, 0.5, 0, 7, 1, 0, 1, 700, 1, hardwareParameter);
	new_goStraightCm_Front_Vision(30,
								  80,
								  100,
								  DETECT_YELLOW_GOAL_SIG,
								  front_vision,
								  0.5,
								  0,
								  1,
								  0.5,
								  0,
								  15,
								  0.25,
								  0,
								  1,
								  600,
								  1,
								  hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(20, angle, 60, MOVE_FORWARD, 1.5, 0, 1.5, 1, 0, 0, 600, 1, hardwareParameter);

	// new_goStraightCm_Front_Vision(40, 80, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
	//   										0.5, 0, 1, 0.3, 0, 4, 0.65, 0, 4, 1000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 5, RELEASE_BRIDGE + 100, 1};
	delay(100);
	//	intakeAction_1 = {90, 400, 0, 90, 1};
	delay(500);

	goStraightCmPID_lib(150, 58, 95, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1550, 1, hardwareParameter); // 1650
	armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	delay(250);

	goStraightCmPID_lib(15, 62, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 500, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};

	////place first yellow goal
	//	delay(300);
	////take first blue goal by front claw
	goStraightCmPID_lib(5, 63, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 300, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 200, 2};
	delay(150);
	armAction_2 = {127, 200, 0, 2};
	goStraightCmPID_lib(70, 55, 127, MOVE_BACKWARD, 5.5, 0, 1.5, 0.5, 0, 5, 1200, 1, hardwareParameter);

	//	intakeAction_1 = {0, 0, 0, 0, 1};
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(27, 55, 100, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 700, 1, hardwareParameter);
	turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	// turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter);
	goStraightCmPID_lib(27, 235, 60, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);

	//  goStraightCm_Front_Vision(30, 239, 100, DETECT_BLUE_GOAL_SIG, front_vision,
	//	  									0.5, 0, 1, 0.3, 0, 7, 0.3, 0, 1, 1000, 1, hardwareParameter);

	delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -250, 1};
	delay(100);
	goStraightCmPID_lib(75, 350, 127, MOVE_BACKWARD, 5.5, 0, 1.2, 0.7, 0, 10, 1800, 1, hardwareParameter); // 1500
	goStraightCm_Back_Vision(25,
							 350,
							 80,
							 DETECT_RED_GOAL_SIG,
							 back_vision,
							 0.5,
							 0,
							 1,
							 0.5,
							 0,
							 5,
							 1,
							 0,
							 1,
							 500,
							 1,
							 hardwareParameter);
	////take first red goal by back claw
	//  delay(100);
	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE + 300, 1};
	intakeAction_1 = {127, 00, 0, 127, 1};
	// waitForTouch();
	goStraightCmPID_lib(102, 347, 90, MOVE_FORWARD, 3.5, 0, 2.5, 1, 0, 0, 2000, 1, hardwareParameter);
	//	goStraightCmPID_lib(45, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 0, 1000, 1, hardwareParameter); //1300
	//	goStraightCmPID_lib(55, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 1300, 2, hardwareParameter);  //1300
	goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 2.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);	  // 1300
	goStraightCmPID_lib(35, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 0, 800, 1, hardwareParameter);	  // 1300
	goStraightCmPID_lib(30, 0, 65, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1300, 2, hardwareParameter);	  // 1300
	goStraightCmPID_lib(20, 0, 75, MOVE_BACKWARD, 3.5, 0, 2.5, 0.5, 0, 5, 600, 1, hardwareParameter); // 1300

	// goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);

	//	goStraightCmPID_lib(60, 0, 100, MOVE_BACKWARD, 1.5, 0, 1, 0.5, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {-20, 0, 200, 0, 1};
	angle = 113;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	// turnDegreesPID_lib(105, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter);

	intakeAction_1 = {127, 0, 0, 127, 1};
	goStraightCmPID_lib(65, angle, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	delay(100);
	goStraightCmPID_lib(5, angle, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 300, 1, hardwareParameter);
	////place blue mobile goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 150, 1};
	delay(200);

	////take middle high yellow goal
	goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {-10, 0, 0, -10, 1};
	armAction_1 = {127, 200, -20, 1};
	turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	intakeAction_1 = {127, 0, 0, 127, 1};
	//	armAction_1 = {127, 0, 20, 1};
	delay(100); //
	new_goStraightCm_Front_Vision(40, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);

	//	intakeAction_1 = {-10, 0, 0, -10, 1};
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 45, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	delay(100);

	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	delay(400);
	intakeAction_1 = {127, 00, 0, 127, 1};

	// move high yellow goal to red bridge
	angle = 274;
	goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 10, 1};
	goStraightCmPID_lib(5, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 350, 1, hardwareParameter);
	delay(200);
	////place high yellow mobile goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 350, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	hookAction_1 = {0, false, 1};
	delay(200);

	goStraightCmPID_lib(45, 270, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(100);
	goStraightCmPID_lib(22, 270, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 10, 800, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(40, 90, 55, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);

	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	delay(500);

	angle = 250;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	// turnDegreesPID_lib(angle, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);
	goStraightCmPID_lib(130, angle, 100, MOVE_FORWARD, 3, 0, 1, 0.4, 0, 15, 1750, 1, hardwareParameter);

	delay(100);
	angle = get_robot_heading_lib(hardwareParameter);
	if (angle >= 275)
	{
		armAction_1 = {127, 300, PRESS_BRIDGE, 1};
		turnDegreesPID_lib(275, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 800, 1, hardwareParameter);
	}
	else
	{
		armAction_1 = {127, 300, PRESS_BRIDGE, 1};
		turnDegreesPID_lib(275, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 800, 1, hardwareParameter);
	}

	// turnDegreesPID_lib(275, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 800, 1, hardwareParameter);
	delay(250);
	////place red mobile goal in red bridge
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 270, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);
	goStraightCmPID_lib(12, angle + 10, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 500, 1, hardwareParameter);
	//	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter); 				//shit
	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	// turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter); 				//shit
	armAction_1 = {127, 0, 0, 1};
	goStraightCmPID_lib(120, 178, 100, MOVE_BACKWARD, 2, 0, 1, 1, 0, 5, 1800, 1, hardwareParameter);
	goStraightCm_Back_Vision(50,
							 180,
							 60,
							 DETECT_BLUE_GOAL_SIG,
							 back_vision,
							 0.5,
							 0,
							 1,
							 0.5,
							 0,
							 5,
							 1,
							 0,
							 1,
							 1200,
							 1,
							 hardwareParameter);
	// take corner blue goal
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	intakeAction_1 = {127, 0, 0, 127, 1};

	goStraightCmPID_lib(50, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 800, 1, hardwareParameter); // 900
	turnDegreesPID_lib(90, ON_SPOT_TURN, 80, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);	   // 900
	goStraightCm_Front_Vision(30,
							  90,
							  90,
							  DETECT_YELLOW_GOAL_SIG,
							  front_vision,
							  0.5,
							  0,
							  1,
							  0.3,
							  0,
							  10,
							  0.4,
							  0,
							  0,
							  800,
							  1,
							  hardwareParameter);

	goStraightCmPID_lib(30, 90, 50, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	//	intakeAction_1 = {127, 300, 0, 127, 1};
	goStraightCmPID_lib(136, 120, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 2100, 1, hardwareParameter); // 2000
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	delay(200);
	goStraightCmPID_lib(15, 140, 127, MOVE_FORWARD, 3.5, 0, 1, 0.4, 0, 0, 500, 1, hardwareParameter);
	////////////////push left
	// release third yellow goal
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 125, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);
	goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 0.35, 0, 5, 1000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(30, 120, 80, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 900, 2, hardwareParameter);
	turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	goStraightCmPID_lib(40, 300, 60, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);

	delay(100);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 260, 1};
	delay(400);
	turnDegreesPID_lib(120, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	// turnDegreesPID_lib(115, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(80, 115, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	//	armAction_1 = {127, 0, PRESS_BRIDGE - 250, 1};
	goStraightCmPID_lib(10, 120, 127, MOVE_FORWARD, 4, 0, 1, 0.5, 0, 0, 400, 1, hardwareParameter);

	// release last blue goal to the bridge
	delay(100);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 150);

	armAction_1 = {127, 300, 400, 1};
	goStraightCmPID_lib(30, 115, 100, MOVE_BACKWARD, 2, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);
	turnDegreesPID_lib(40, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	//	goStraightCmPID_lib(15, 30, 100, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 0, 600, 1, hardwareParameter);
	//  delay(400);
	armAction_1 = {127, 300, 200, 1};
	goStraightCmPID_lib(90, 40, 90, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 8, 1600, 1, hardwareParameter);
	//	goStraightCmPID_lib(15, 90, 90, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 8, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, 500, 1};
	turnDegreesPID_lib(335, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	// turnDegreesPID_lib(315, ON_SPOT_TURN, 100, CLOCKWISE, 1.2, 0, -1, 800, 2, hardwareParameter);

	double distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 200);
	if (distance > 50 || distance < 20)
		distance = 40;

	goStraightCm_Back_Vision(distance + 3,
							 345,
							 50,
							 DETECT_RED_GOAL_SIG,
							 back_vision,
							 0.5,
							 0,
							 1,
							 0.5,
							 0,
							 5,
							 5,
							 0,
							 1,
							 1000,
							 1,
							 hardwareParameter); // 40
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	//	turnDegreesPID_lib(290, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	intakeAction_1 = {127, 200, 0, 127, 1};

	goStraightCmPID_lib(100, 285, 127, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 2000, 1, hardwareParameter);
	long time_left = 60000 - (pros::millis() - start_time);
	hookAction_1 = {time_left - 200, false, 1};
	//	goStraightCmPID_lib(160, 280, 65, MOVE_FORWARD,	1.5, 0, 0, 1, 0, 0, 3500, 1, hardwareParameter);

	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);

	goStraightCmPID_lib(110, 270, 127, MOVE_FORWARD, 3, 0, 0, 2, 0, 0, 800, 1, hardwareParameter);
	// if(pros::millis() - start_time < 58800){
	// 	delay(300);
	// }
	//	goStraightCmPID_lib(80, 280, 75, MOVE_FORWARD,	1, 0, 1, 0.5, 0, 0, 2000, 1, hardwareParameter);

	std::cout << "Time Left: " << pros::millis() - start_time << std::endl;
}

void auton_60s_skills_fast_version()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long hook_action_delay_time = 50;
	long claw_action_delay_time = 75;
	int intake_speed = 110;
	long start_time = pros::millis();
	pros::Task arm_holding_task(arm_moving_holding_fn, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm_holding_task");

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, false, 1};
	delay(30);

	// 	hookAction_1 = {0, true, 1};
	// 	turnDegreesPID_lib(90, ONE_WHEEL_TURN_FORWARD, 70, CLOCKWISE, 6, 0, -20, 1000, 1, hardwareParameter);
	// waitForTouch();

	// ------------------------------------------------------------------------------------ //
	//// part 1 take first blue goal
	goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);

	intakeAction_1 = {100, 500, 0, 100, 1};
	armAction_1 = {70, 400, 0, 1};
	goStraightCmPID_lib(90, 77, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter); // 100,80,127
	// arm_motor.move(0);
	// arm_motor.tare_position();

	new_goStraightCm_Front_Vision(30, 77, 100,
								  DETECT_YELLOW_GOAL_SIG,
								  front_vision,
								  0.5,
								  0,
								  1,
								  0.5,
								  0,
								  15,
								  0.25,
								  0,
								  1,
								  600,
								  1,
								  hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(15, angle, 60, MOVE_FORWARD, 1.5, 0, 1.5, 0.8, 0, 0, 600, 1, hardwareParameter);

	///////catch first yellow goal
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, RELEASE_BRIDGE - 100, 1};
	goStraightCmPID_lib(38, 25, 100, MOVE_BACKWARD, 2.0, 0, 2.5, 1, 0, 5, 800, 1, hardwareParameter); // 100,80,127
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	//////catch first line of rings
	goStraightCmPID_lib(93, 90, 70, MOVE_FORWARD, 3.0, 0, 1.5, 0.3, 0, 10, 2000, 1, hardwareParameter); // 100,80,127
	// intakeAction_1 = {127, 0, 0, 127, 1};
	delay(200);
	goStraightCmPID_lib(30, 30, 100, MOVE_BACKWARD, 1.0, 0, 1.5, 0.5, 0, 5, 1000, 1, hardwareParameter); // 100,80,127

	// waitForTouch();

	armAction_1 = {127, 0, RELEASE_BRIDGE, 1};
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	goStraightCmPID_lib(65, 30, 127, MOVE_FORWARD, 2.5, 0, 1.5, 0.5, 0, 2, 1200, 1, hardwareParameter); // 100,80,127
	armAction_1 = {127, 500, RELEASE_BRIDGE - 100, 1};
	goStraightCmPID_lib(65, 60, 127, MOVE_FORWARD, 2.5, 0, 1.5, 0.5, 0, 10, 1200, 1, hardwareParameter); // 100,80,127
	armAction_1 = {127, 0, PRESS_BRIDGE - 25, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	angle = 70;
	goStraightCmPID_lib(10, angle, 127, MOVE_FORWARD, 2.5, 0, 1.5, 0.5, 0, 10, 350, 1, hardwareParameter); // 100,80,127
	///////place the first yellow goal to the bridge
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	// armAction_1 = {127, 300, PRESS_BRIDGE + 100, 1};
	goStraightCmPID_lib(3, angle, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 10, 0, 5, 300, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(100);
	goStraightCmPID_lib(70, 50, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 5, 1200, 1, hardwareParameter);

	// waitForTouch();

	////take first blue goal by front claw
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(27, 50, 100, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 700, 1, hardwareParameter);
	turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	goStraightCmPID_lib(27, 230, 60, MOVE_FORWARD, 3.5, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);
	delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, 350, 1};
	delay(100);

	goStraightCmPID_lib(65, 5, 127, MOVE_BACKWARD, 1.0, 0, 1.2, 0.7, 0, 10, 1000, 1, hardwareParameter); // 1500
	double distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 200);
	if (distance > 70 || distance < 10)
		distance = 20;

	goStraightCm_Back_Vision(distance + 20,
							 350,
							 65,
							 DETECT_RED_GOAL_SIG,
							 back_vision,
							 0.5,
							 0,
							 1,
							 0.5,
							 0,
							 5,
							 0.5,
							 0,
							 5,
							 1000,
							 1,
							 hardwareParameter);
	//    goStraightCmPID_lib(5, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 800, 1, hardwareParameter); //900

	////take first red goal by back claw
	//  delay(100);
	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE + 300, 1};
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	////catch first five flower ring
	goStraightCmPID_lib(112, 340, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 5, 1600, 1, hardwareParameter);

	//  goStraightCmPID_lib(37, 5, 60, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 1200, 1, hardwareParameter); //1300
	goStraightCmPID_lib(20, 10, 60, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); // 1300
	goStraightCmPID_lib(25, 60, 75, MOVE_FORWARD, 2.5, 0, 0, 0.8, 0, 2, 800, 1, hardwareParameter); // 1300
	//  turnDegreesPID_lib(60, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 1, hardwareParameter);
	goStraightCmPID_lib(43, 85, 75, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 1100, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	goStraightCmPID_lib(15, 90, 100, MOVE_FORWARD, 2.5, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(3, 100, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 10, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 200, 1};
	delay(100);
	goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);

	//	intakeAction_1 = {-10, 0, 0, -10, 1};
	armAction_1 = {127, 200, 0, 1};
	turnDegreesPID_lib(245, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};

	////take middle high yellow goal
	delay(100); //
	new_goStraightCm_Front_Vision(30, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 1, 1000, 1, hardwareParameter);
	//	intakeAction_1 = {-10, 0, 0, -10, 1};
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(25, angle, 45, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	//		delay(100);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	delay(100);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	angle = 270;
	goStraightCmPID_lib(120, angle, 75, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 1900, 1, hardwareParameter);

	// move high yellow goal to red bridge
	armAction_1 = {127, 0, PRESS_BRIDGE - 10, 1};
	goStraightCmPID_lib(10, angle, 100, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 450, 1, hardwareParameter);
	delay(200);
	////place high yellow mobile goal
	armAction_1 = {127, 0, PRESS_BRIDGE - 20, 1};
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 100);
	goStraightCmPID_lib(5, angle, 127, MOVE_BACKWARD, 3.5, 0, 2.5, 4, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);
	hookAction_1 = {0, false, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};

	////move backward to catch the red goal by using front claw
	goStraightCmPID_lib(45, 270, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	delay(100);
	goStraightCmPID_lib(22, 270, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 10, 700, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(30, 90, 65, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	// catch red goal by front claw
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);

	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};
	//		delay(100);
	angle = 250;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 70, COUNTER_CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	goStraightCmPID_lib(100, angle, 100, MOVE_FORWARD, 3, 0, 1, 0.5, 0, 8, 1750, 1, hardwareParameter);

	armAction_1 = {127, 300, PRESS_BRIDGE, 1};
	delay(150); // 250
	double tmp_angle = getNormalizedAngle(get_robot_heading_lib(hardwareParameter));
	if (tmp_angle < 270)
		turnDegreesPID_lib(270, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, -20, 700, 1, hardwareParameter);

	else
		turnDegreesPID_lib(270, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 700, 1, hardwareParameter);

	////place red mobile goal in red bridge
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);

	goStraightCmPID_lib(5, 270, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(100);
	goStraightCmPID_lib(12, angle + 10, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 500, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	armAction_1 = {127, 0, 100, 1};
	goStraightCmPID_lib(120, 178, 127, MOVE_BACKWARD, 2, 0, 1, 0.3, 0, 5, 1500, 1, hardwareParameter);

	// distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 200);
	// if(distance > 70 || distance < 10){
	// 	distance = 50;
	// }
	// goStraightCm_Back_Vision(distance + 5, 180, 60, DETECT_BLUE_GOAL_SIG, back_vision,
	// 	                       0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 1, 1200, 1, hardwareParameter);

	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////
	//	armAction_1 = {127, 0, 100, 1};
	distance = get_distance_back_vision(back_vision, DETECT_BLUE_GOAL_SIG, 10, 10, 60, 200);
	if (distance > 70 || distance < 10)
		distance = 50;

	goStraightCm_Back_Vision(distance + 20, 180, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 0.5, 0, 1, 1200, 1, hardwareParameter);
	// take corner blue goal
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	intakeAction_1 = {intake_speed, 0, 0, intake_speed, 1};
	///////////////////////////////////////?????
	//	goStraightCmPID_lib(30, 90, 127, MOVE_FORWARD, 6, 0, 1, 0.4, 0, 5, 1000, 1, hardwareParameter); //900
	goStraightCmPID_lib(3, 180, 100, MOVE_FORWARD, 1, 0, 1, 5, 0, 3, 600, 1, hardwareParameter); // 900
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	goStraightCmPID_lib(70, 90, 70, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 1500, 1, hardwareParameter);	  // 900
	goStraightCmPID_lib(40, 120, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 5, 1000, 1, hardwareParameter); // 900
	armAction_1 = {127, 0, 0, 1};
	goStraightCm_Front_Vision(20, 115, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 500, 1, hardwareParameter);

	//	  goStraightCmPID_lib(30, 90, 50, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	//	delay(300);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(25, angle, 45, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 700, 1, hardwareParameter);

	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 300, 1};

	//	goStraightCmPID_lib(60, 110, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.3, 0, 5, 1100, 1, hardwareParameter); //2000
	goStraightCmPID_lib(130, 125, 70, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 2500, 1, hardwareParameter); // 2000
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	delay(200);
	goStraightCmPID_lib(15, 140, 127, MOVE_FORWARD, 3.5, 0, 1, 0.4, 0, 0, 500, 1, hardwareParameter);

	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time);
	goStraightCmPID_lib(5, 125, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 100, 1};
	delay(200);
	goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 0.35, 0, 5, 1000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(hook_action_delay_time);
	goStraightCmPID_lib(30, 120, 80, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 900, 2, hardwareParameter);
	turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, -20, 1100, 2, hardwareParameter);
	goStraightCmPID_lib(35, 300, 60, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);

	delay(100);
	clawAction_1 = {0, true, 1};
	delay(claw_action_delay_time);
	armAction_1 = {127, 0, PRESS_BRIDGE + 260, 1};
	turnDegreesPID_lib(120, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, -20, 1300, 2, hardwareParameter);

	goStraightCmPID_lib(75, 115, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	//	armAction_1 = {127, 0, PRESS_BRIDGE + 250, 1};
	goStraightCmPID_lib(10, 120, 127, MOVE_FORWARD, 4, 0, 1, 0.5, 0, 0, 400, 1, hardwareParameter);

	// release last blue goal to the bridge
	delay(100);
	clawAction_1 = {0, false, 1};
	delay(claw_action_delay_time + 150);

	armAction_1 = {127, 300, 400, 1};
	goStraightCmPID_lib(30, 115, 100, MOVE_BACKWARD, 2, 0, 1, 0.3, 0, 0, 1000, 1, hardwareParameter);
	turnDegreesPID_lib(40, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 1000, 2, hardwareParameter);
	//	goStraightCmPID_lib(15, 30, 100, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 0, 600, 1, hardwareParameter);

	////////////////////////////////////////
	////////////////////////////////////////
	////////////////////////////////////////

	armAction_1 = {127, 300, 200, 1};
	//	intakeAction_1 = {intake_speed, 200, 400, intake_speed, 1};
	goStraightCmPID_lib(90, 40, 127, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 4, 1100, 1, hardwareParameter);
	//	goStraightCmPID_lib(15, 90, 90, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 8, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, 500, 1};
	turnDegreesPID_lib(330, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, -20, 600, 2, hardwareParameter);
	// turnDegreesPID_lib(315, ON_SPOT_TURN, 100, CLOCKWISE, 1.2, 0, -1, 800, 2, hardwareParameter);

	distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 200);
	if (distance > 50 || distance < 20)
		distance = 40;

	goStraightCm_Back_Vision(distance + 12, 345, 50, DETECT_RED_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 5, 0, 1, 900, 1, hardwareParameter); // 40
	//  delay(100);
	hookAction_1 = {0, true, 1};
	delay(hook_action_delay_time);
	//	turnDegreesPID_lib(290, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, -20, 800, 2, hardwareParameter);
	intakeAction_1 = {127, 0, 0, 127, 1};

	goStraightCmPID_lib(100, 285, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 2000, 1, hardwareParameter);

	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	goStraightCmPID_lib(100, 270, 127, MOVE_FORWARD, 3, 0, 0, 10, 0, 0, 3000, 1, hardwareParameter);

	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	waitForTouch();
}

void right_side_red() // slot 2
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	clawAction_1 = {950, true, 1};
	goStraightCmPID_lib(87, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 2, 5000, 1, hardwareParameter);
	delay(50);
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, -10, 1};
	goStraightCmPID_lib(65, 100, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 3, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	if (true)
	{ // drop current goal whether or not you have it. go for middle goal
		armAction_1 = {127, 0, 50, 1};
		delay(200);
		turnDegreesPID_lib(350, ON_SPOT_TURN, 127, CLOCKWISE, 2, 0, 0, 2000, 1, hardwareParameter);
		delay(100);
		clawAction_1 = {0, false, 1};
		goStraightCmPID_lib(10, 350, 80, MOVE_FORWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		goStraightCmPID_lib(5, 350, 80, MOVE_BACKWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		armAction_1 = {127, 0, 200, 1};

		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		goStraightCm_Back_Vision(90, 180, 100, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1800, 1, hardwareParameter);
		armAction_1 = {125, 0, 300, 1};
		hookAction_1 = {0, true, 1};
		intakeAction_1 = {127, 300, 10000, 127, 1};
		delay(100);

		goStraightCmPID_lib(10, 180, 80, MOVE_FORWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		goStraightCmPID_lib(100, 135, 80, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);

		delay(20);
		// bool f = false;
		// for (int i = 0; i < 20; i++)
		// {
		// 	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
		// 	if (closest_goal.width >= 90)
		// 	{
		// 		pros::lcd::print(0, "width: %d", closest_goal.width);
		// 		f = true;
		// 		break;
		// 	}
		// 	break;
		// 	delay(10);
		// }

		if (true)
		{
			delay(200);
			goStraightCm_Front_Vision(30, 135, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1000, 1, hardwareParameter);
			double angle = get_robot_heading_lib(hardwareParameter);
			goStraightCmPID_lib(25, angle, 80, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
			clawAction_1 = {0, true, 1};
			goStraightCmPID_lib(45, 135, 127, MOVE_BACKWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
			armAction_1 = {125, 0, 300, 1};
			turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 1, hardwareParameter);
			goStraightCmPID_lib(120, 180, 90, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
			hookAction_1 = {500, false, 1};
			goStraightCmPID_lib(120, 160, 127, MOVE_BACKWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
		}
		else
		{
			turnDegreesPID_lib(175, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 2, 0, 0, 1200, 1, hardwareParameter);
			goStraightCmPID_lib(180, 175, 70, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 2000, 1, hardwareParameter);
			goStraightCmPID_lib(25, 175, 100, MOVE_BACKWARD, 1, 0, 1, 10, 0, 2, 1200, 1, hardwareParameter);
			turnDegreesPID_lib(145, ON_SPOT_TURN, 127, CLOCKWISE, 2, 0, 0, 1200, 1, hardwareParameter);
			delay(100);
			closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
			std::cout << closest_goal.width << std::endl;

			if (closest_goal.width >= 60)
			{
				armAction_1 = {127, 0, 0, 1};
				delay(200);
				goStraightCm_Front_Vision(40, 145, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1000, 1, hardwareParameter);
				double angle = get_robot_heading_lib(hardwareParameter);
				goStraightCmPID_lib(20, angle, 90, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
				clawAction_1 = {0, true, 1};
				armAction_1 = {125, 0, 200, 1};
				goStraightCmPID_lib(170, 160, 127, MOVE_BACKWARD, 1, 0, 0, 100, 0, 0, 5000, 1, hardwareParameter);
			}
			else
			{
				goStraightCmPID_lib(170, 160, 127, MOVE_BACKWARD, 1, 0, 0, 100, 0, 0, 5000, 1, hardwareParameter);
			}

			std::cout << pros::millis() - startingtime << std::endl;
		}
	}
	//	else if ((pros::millis() - startingtime) < 3000 && (pros::millis() - startingtime) <= 6000){ // go for win point or mid depending on qual or final

	std::cout << pros::millis() - startingtime << std::endl;
	waitForTouch();
}

void right_side_red_2() // slot 3, uploaded as right_side_red_2
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(120, 125, 127, MOVE_FORWARD, 2, 0, 1, 10, 0, 0, 1000, 1, hardwareParameter);
	armAction_1 = {125, 0, -30, 1};
	goStraightCm_Front_Vision(40, 125, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(20, angle, 80, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 500, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	std::cout << "time at grab: " << pros::millis() - startingtime << std::endl;
	armAction_1 = {125, 0, 20, 1};
	goStraightCmPID_lib(65, 140, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 15000, 1, hardwareParameter);

	hookAction_1 = {0, false, 1};
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 1, hardwareParameter);
	goStraightCm_Back_Vision(80, 180, 100, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1200, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(50, 250, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	clawAction_1 = {100, false, 1};
	goStraightCmPID_lib(20, angle, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 700, 1, hardwareParameter);
	goStraightCmPID_lib(10, angle, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 700, 1, hardwareParameter);
	armAction_1 = {125, 0, 200, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
	goStraightCmPID_lib(23, 90, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1000, 1, hardwareParameter);

	delay(100);
	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
	std::cout << "Yellow width: " << closest_goal.width << std::endl;

	if (closest_goal.width >= 90)
	{
		armAction_1 = {125, 0, -80, 1};
		goStraightCm_Front_Vision(40, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
		angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(40, angle, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
		clawAction_1 = {100, true, 1};
		delay(100);
		armAction_1 = {125, 200, 400, 1};
		goStraightCmPID_lib(40, 90, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 1, hardwareParameter);
		goStraightCmPID_lib(50, 180, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
		intakeAction_1 = {127, 0, 0, 127, 1};
		goStraightCmPID_lib(150, 180, 80, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1800, 1, hardwareParameter);
		delay(250);
		goStraightCmPID_lib(150, 165, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 2000, 1, hardwareParameter);
	}
	else
	{
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
		intakeAction_1 = {127, 0, 0, 127, 1};
		goStraightCmPID_lib(50, 180, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
		goStraightCmPID_lib(110, 180, 85, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 2000, 1, hardwareParameter);
		goStraightCmPID_lib(25, 180, 100, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
		turnDegreesPID_lib(155, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, 0, 1000, 1, hardwareParameter);

		delay(100);

		closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
		std::cout << "2nd Yellow width: " << closest_goal.width << std::endl;

		if (closest_goal.width >= 30)
		{
			armAction_1 = {125, 0, -80, 1};
			goStraightCm_Front_Vision(30, 155, 127, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
			angle = get_robot_heading_lib(hardwareParameter);
			goStraightCmPID_lib(35, angle, 83, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
			clawAction_1 = {100, true, 1};
			delay(100);
			armAction_1 = {125, 200, 400, 1};
			goStraightCmPID_lib(70, angle, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);

			std::cout << "Cross the line time: " << pros::millis() - startingtime << std::endl;

			turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
			hookAction_1 = {0, false, 1};
		}

		else
			hookAction_1 = {0, false, 1};

		goStraightCmPID_lib(150, 170, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
	}

	std::cout << "End time: " << pros::millis() - startingtime << std::endl;
}

void right_side_red_3() // go straight for mid
{
	sys_initial_robot_heading = 135;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	clawAction_1 = {1100, true, 1};
	armAction_1 = {125, 0, -40, 1};
	goStraightCmPID_lib(140, 135, 127, MOVE_FORWARD, 2, 0, 1, 10, 0, 0, 2500, 1, hardwareParameter);
	delay(150);
	armAction_1 = {125, 0, 70, 1};
	goStraightCmPID_lib(55, 135, 127, MOVE_BACKWARD, 2, 0, 1, 10, 0, 0, 15000, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
	goStraightCm_Back_Vision(120, 180, 100, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 2500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {127, 200, 0, 127, 1};
	goStraightCmPID_lib(30, 200, 127, MOVE_FORWARD, 2, 0, 1, 10, 0, 0, 5000, 1, hardwareParameter);
}

void left_side_red() // setup face a bit to the right
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 85;
	long startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	clawAction_1 = {950, true, 1};
	goStraightCmPID_lib(107, 85, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 2, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	std::cout << "time at grab: " << pros::millis() - startingtime << std::endl;
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(125, 85, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 1, hardwareParameter);
	std::cout << "time after: " << pros::millis() - startingtime << std::endl;

	hookAction_1 = {0, false, 1};
	turnDegreesPID_lib(165, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 1, hardwareParameter);
	goStraightCmPID_lib(15, 165, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	goStraightCm_Back_Vision(35, 165, 70, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(50);

	goStraightCmPID_lib(35, 135, 127, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 1500, 1, hardwareParameter);
	turnDegreesPID_lib(225, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1000, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(10, 225, 127, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	goStraightCmPID_lib(10, 225, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	turnDegreesPID_lib(52, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, -1, 1500, 1, hardwareParameter);
	armAction_1 = {127, 0, 200, 1};
	delay(50);
	intakeAction_1 = {127, 0, 0, 127, 1};
	goStraightCmPID_lib(125, 52, 50, MOVE_FORWARD, 2, 0, 2, 5, 0, 3, 5000, 1, hardwareParameter);

	delay(100);

	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
	std::cout << "Yellow width: " << closest_goal.width << std::endl;

	if (closest_goal.width >= 120)
	{
		goStraightCmPID_lib(20, 52, 60, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 700, 1, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		intakeAction_1 = {0, 0, 0, 0, 1};
		goStraightCm_Front_Vision(30, 52, 80, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1200, 1, hardwareParameter);
		clawAction_1 = {0, true, 1};
		goStraightCmPID_lib(120, 52, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 2000, 1, hardwareParameter);
	}
	else
	{
		goStraightCmPID_lib(100, 52, 100, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 2000, 1, hardwareParameter);
	}
}

void Red_Double_WP()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long start_time = pros::millis();
	vision_object_s_t closest_goal;

	// ------------------------------------------------------------------------------------ //
	// part 1, take the first red goal
	goStraightCmPID_lib(3.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};

	goStraightCmPID_lib(90, 80, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 2500, 1, hardwareParameter); // 100,80,127
	delay(200);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, -1, 1000, 1, hardwareParameter);
	intakeAction_1 = {127, 0, 0, 127, 1};
	goStraightCmPID_lib(170, 2, 70, MOVE_FORWARD, 2.0, 0, 2.5, 5, 0, 0, 4000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	turnDegreesPID_lib(135, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, 0, 1200, 1, hardwareParameter);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(70, 135, 127, MOVE_BACKWARD, 2.0, 0, 2.5, 0.4, 0, 3, 1800, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(100);
	intakeAction_1 = {0, 0, 0, 0, 1};

	// part 2

	goStraightCmPID_lib(40, 135, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 800, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, 0, 1200, 1, hardwareParameter);
	goStraightCm_Back_Vision(80, 180, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(100);
	intakeAction_1 = {127, 100, 0, 127, 1};
	armAction_1 = {127, 0, 200, 1};
	goStraightCmPID_lib(10, 180, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 1500, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, 0, 1200, 1, hardwareParameter);
	goStraightCmPID_lib(90, 90, 70, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 1000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(50, 90, 127, MOVE_BACKWARD, 2.0, 0, 2.5, 1, 0, 0.001, 700, 1, hardwareParameter);

	std::cout << "Ending time: " << pros::millis() - start_time << std::endl;
}

void right_side_blue() // slot 2
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	clawAction_1 = {950, true, 1};
	goStraightCmPID_lib(107, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 2, 5000, 1, hardwareParameter);
	delay(50);
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, -10, 1};
	goStraightCmPID_lib(100, 100, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 3, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	if (true)
	{ // drop current goal whether or not you have it. go for middle goal
		armAction_1 = {127, 0, 50, 1};
		delay(200);
		turnDegreesPID_lib(350, ON_SPOT_TURN, 127, CLOCKWISE, 2, 0, 0, 2000, 1, hardwareParameter);
		delay(100);
		clawAction_1 = {0, false, 1};
		goStraightCmPID_lib(10, 350, 80, MOVE_FORWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		goStraightCmPID_lib(5, 350, 80, MOVE_BACKWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		armAction_1 = {127, 0, 200, 1};

		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		goStraightCm_Back_Vision(90, 180, 100, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1800, 1, hardwareParameter);
		armAction_1 = {125, 0, 300, 1};
		hookAction_1 = {0, true, 1};
		intakeAction_1 = {127, 300, 10000, 127, 1};
		delay(100);

		goStraightCmPID_lib(100, 135, 80, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);

		delay(20);
		// bool f = false;
		// for (int i = 0; i < 20; i++)
		// {
		// 	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
		// 	if (closest_goal.width >= 90)
		// 	{
		// 		pros::lcd::print(0, "width: %d", closest_goal.width);
		// 		f = true;
		// 		break;
		// 	}
		// 	break;
		// 	delay(10);
		// }

		if (true)
		{
			armAction_1 = {127, 0, 0, 1};
			delay(200);
			goStraightCm_Front_Vision(30, 135, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1000, 1, hardwareParameter);
			double angle = get_robot_heading_lib(hardwareParameter);
			goStraightCmPID_lib(25, angle, 80, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
			clawAction_1 = {0, true, 1};
			goStraightCmPID_lib(45, 135, 127, MOVE_BACKWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
			armAction_1 = {125, 0, 300, 1};
			turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 1, hardwareParameter);
			goStraightCmPID_lib(120, 180, 90, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
			hookAction_1 = {500, false, 1};
			goStraightCmPID_lib(120, 160, 127, MOVE_BACKWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
		}
		else
		{
			turnDegreesPID_lib(175, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 2, 0, 0, 1200, 1, hardwareParameter);
			goStraightCmPID_lib(180, 175, 70, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 2000, 1, hardwareParameter);
			goStraightCmPID_lib(25, 175, 100, MOVE_BACKWARD, 1, 0, 1, 10, 0, 2, 1200, 1, hardwareParameter);
			turnDegreesPID_lib(145, ON_SPOT_TURN, 127, CLOCKWISE, 2, 0, 0, 1200, 1, hardwareParameter);
			delay(100);
			closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
			std::cout << closest_goal.width << std::endl;

			if (closest_goal.width >= 60)
			{
				armAction_1 = {127, 0, 0, 1};
				delay(200);
				goStraightCm_Front_Vision(40, 145, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1000, 1, hardwareParameter);
				double angle = get_robot_heading_lib(hardwareParameter);
				goStraightCmPID_lib(20, angle, 90, MOVE_FORWARD, 1, 0, 1, 10, 0, 2, 1500, 1, hardwareParameter);
				clawAction_1 = {0, true, 1};
				armAction_1 = {125, 0, 200, 1};
				goStraightCmPID_lib(170, 160, 127, MOVE_BACKWARD, 1, 0, 0, 100, 0, 0, 5000, 1, hardwareParameter);
			}
			else
			{
				goStraightCmPID_lib(170, 160, 127, MOVE_BACKWARD, 1, 0, 0, 100, 0, 0, 5000, 1, hardwareParameter);
			}

			std::cout << pros::millis() - startingtime << std::endl;
		}
	}
	//	else if ((pros::millis() - startingtime) < 3000 && (pros::millis() - startingtime) <= 6000){ // go for win point or mid depending on qual or final

	//	}
	else
	{ // keep pulling
		turnDegreesPID_lib(210, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 10000, 1, hardwareParameter);
		armAction_1 = {127, 0, 100, 1};
		goStraightCm_Back_Vision(40, 225, 80, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(25, 225, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	}

	std::cout << pros::millis() - startingtime << std::endl;
	waitForTouch();
}

void right_side_blue_2() // slot 3, uploaded as right_side_red_2
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(120, 125, 127, MOVE_FORWARD, 2, 0, 1, 10, 0, 0, 1000, 1, hardwareParameter);
	armAction_1 = {125, 0, -30, 1};
	goStraightCm_Front_Vision(40, 125, 100, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(20, angle, 80, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 500, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	std::cout << "time at grab: " << pros::millis() - startingtime << std::endl;
	armAction_1 = {125, 0, 20, 1};
	goStraightCmPID_lib(65, 140, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 15000, 1, hardwareParameter);

	hookAction_1 = {0, false, 1};
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 1, hardwareParameter);
	goStraightCm_Back_Vision(80, 180, 100, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1200, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(50, 250, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	clawAction_1 = {100, false, 1};
	goStraightCmPID_lib(20, angle, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 700, 1, hardwareParameter);
	goStraightCmPID_lib(10, angle, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 700, 1, hardwareParameter);
	armAction_1 = {125, 0, 200, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
	goStraightCmPID_lib(23, 90, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1000, 1, hardwareParameter);

	delay(100);
	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
	std::cout << "Yellow width: " << closest_goal.width << std::endl;

	if (closest_goal.width >= 70)
	{
		armAction_1 = {125, 0, -80, 1};
		goStraightCm_Front_Vision(40, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
		angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(40, angle, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
		clawAction_1 = {100, true, 1};
		delay(100);
		armAction_1 = {125, 200, 400, 1};
		goStraightCmPID_lib(40, 90, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 1, hardwareParameter);
		goStraightCmPID_lib(50, 180, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
		intakeAction_1 = {127, 0, 0, 127, 1};
		goStraightCmPID_lib(150, 180, 80, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1800, 1, hardwareParameter);
		delay(250);
		hookAction_1 = {0, false, 1};
		goStraightCmPID_lib(150, 165, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 2000, 1, hardwareParameter);
	}
	else
	{
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
		intakeAction_1 = {127, 0, 0, 127, 1};
		goStraightCmPID_lib(50, 180, 127, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
		goStraightCmPID_lib(140, 180, 85, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 2000, 1, hardwareParameter);
		goStraightCmPID_lib(30, 180, 100, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
		turnDegreesPID_lib(140, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, 0, 1200, 1, hardwareParameter);

		delay(100);

		closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
		std::cout << "2nd Yellow width: " << closest_goal.width << std::endl;

		if (closest_goal.width >= 30)
		{
			armAction_1 = {125, 0, -80, 1};
			goStraightCm_Front_Vision(30, 140, 127, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
			angle = get_robot_heading_lib(hardwareParameter);
			goStraightCmPID_lib(35, angle, 83, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
			clawAction_1 = {100, true, 1};
			delay(100);
			armAction_1 = {125, 200, 400, 1};
			goStraightCmPID_lib(70, angle, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);

			std::cout << "Cross the line time: " << pros::millis() - startingtime << std::endl;

			hookAction_1 = {0, false, 1};
			turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
		}

		else
			hookAction_1 = {0, false, 1};

		hookAction_1 = {0, false, 1};
		goStraightCmPID_lib(150, 180, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
	}

	std::cout << "End time: " << pros::millis() - startingtime << std::endl;
}

void right_side_blue_3() // go straight for mid
{
	sys_initial_robot_heading = 135;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	clawAction_1 = {1100, true, 1};
	armAction_1 = {125, 0, -40, 1};
	goStraightCmPID_lib(140, 135, 127, MOVE_FORWARD, 2, 0, 1, 10, 0, 0, 2500, 1, hardwareParameter);
	delay(150);
	goStraightCmPID_lib(55, 135, 127, MOVE_BACKWARD, 2, 0, 1, 10, 0, 0, 2000, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 2, hardwareParameter);
	goStraightCm_Back_Vision(120, 180, 100, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 2500, 1, hardwareParameter);
	armAction_1 = {125, 0, 70, 1};
	hookAction_1 = {0, true, 1};
	intakeAction_1 = {127, 200, 0, 127, 1};
	goStraightCmPID_lib(30, 200, 127, MOVE_FORWARD, 2, 0, 1, 10, 0, 0, 5000, 1, hardwareParameter);
}

void left_side_blue() // setup face a bit to the right
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 85;
	long startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	clawAction_1 = {950, true, 1};
	goStraightCmPID_lib(107, 85, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 2, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	std::cout << "time at grab: " << pros::millis() - startingtime << std::endl;
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(125, 85, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 1, hardwareParameter);
	std::cout << "time after: " << pros::millis() - startingtime << std::endl;

	hookAction_1 = {0, false, 1};
	turnDegreesPID_lib(165, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 1, hardwareParameter);
	goStraightCmPID_lib(15, 165, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	goStraightCm_Back_Vision(35, 165, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(50);

	goStraightCmPID_lib(35, 135, 127, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 1500, 1, hardwareParameter);
	turnDegreesPID_lib(225, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1000, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	goStraightCmPID_lib(10, 225, 127, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	goStraightCmPID_lib(10, 225, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 500, 1, hardwareParameter);
	turnDegreesPID_lib(52, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, -1, 1500, 1, hardwareParameter);
	armAction_1 = {127, 0, 200, 1};
	delay(50);
	intakeAction_1 = {127, 0, 0, 127, 1};
	goStraightCmPID_lib(125, 52, 50, MOVE_FORWARD, 2, 0, 2, 5, 0, 3, 5000, 1, hardwareParameter);

	delay(100);

	closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
	std::cout << "Yellow width: " << closest_goal.width << std::endl;

	if (closest_goal.width >= 120)
	{
		goStraightCmPID_lib(20, 52, 60, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 700, 1, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		intakeAction_1 = {0, 0, 0, 0, 1};
		goStraightCm_Front_Vision(30, 52, 80, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1200, 1, hardwareParameter);
		clawAction_1 = {0, true, 1};
		goStraightCmPID_lib(120, 52, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 2000, 1, hardwareParameter);
	}
	else
	{
		goStraightCmPID_lib(100, 52, 100, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 2000, 1, hardwareParameter);
	}
}

void Blue_Double_WP()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long start_time = pros::millis();
	vision_object_s_t closest_goal;

	// ------------------------------------------------------------------------------------ //
	// part 1, take the first red goal
	goStraightCmPID_lib(3.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};

	goStraightCmPID_lib(90, 80, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 2500, 1, hardwareParameter); // 100,80,127
	delay(200);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, -1, 1000, 1, hardwareParameter);
	intakeAction_1 = {127, 0, 0, 127, 1};
	hookAction_1 = {1500, false, 1};
	goStraightCmPID_lib(170, 2, 60, MOVE_FORWARD, 2.0, 0, 2.5, 5, 0, 0, 4000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	turnDegreesPID_lib(135, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, 0, 1200, 1, hardwareParameter);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(30, 135, 127, MOVE_BACKWARD, 2.0, 0, 2.5, 0.4, 0, 3, 1800, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(100);
	intakeAction_1 = {0, 0, 0, 0, 1};

	// part 2

	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, 0, 1200, 1, hardwareParameter);
	goStraightCm_Back_Vision(80, 180, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(100);
	intakeAction_1 = {127, 100, 0, 127, 1};
	armAction_1 = {127, 0, 200, 1};
	goStraightCmPID_lib(10, 180, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 1500, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, 0, 1200, 1, hardwareParameter);
	goStraightCmPID_lib(90, 90, 70, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 1000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(50, 90, 127, MOVE_BACKWARD, 2.0, 0, 2.5, 1, 0, 0.001, 700, 1, hardwareParameter);

	std::cout << "Ending time: " << pros::millis() - start_time << std::endl;
}

void test()
{
	vision_object_s_t closest_goal;
	double distance;
	double front_distance;
	int cnt = 0;

	while (true)
	{
		for (int i = 0; i < 10; i++)
		{
			closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
			delay(10);
			if (closest_goal.width >= 120)
				break;
		}

		pros::lcd::print(2, "witdh = %d", closest_goal.width);
		waitForTouch();
	}
	// while (true)
	// {
	// 	cnt++;
	// 	front_distance = get_distance_front_vision(front_vision, DETECT_YELLOW_GOAL_SIG, 10, 10, 60, 200);
	// 	distance = get_distance_back_vision(back_vision, DETECT_RED_GOAL_SIG, 10, 10, 60, 200);
	// 	pros::lcd::print(1, "f_d: %f", front_distance);
	// 	pros::lcd::print(0, "distance: %f, %d", distance, cnt);
	// 	delay(3000);
	// }
}

/**************************

AUTON

**************************/
void autonomous()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	left_back_motor.set_zero_position(0.0);
	left_front_motor.set_zero_position(0.0);
	left_mid_motor.set_zero_position(0.0);
	right_back_motor.set_zero_position(0.0);
	right_front_motor.set_zero_position(0.0);
	right_mid_motor.set_zero_position(0.0);
	intake_motor.set_zero_position(0.0);
	arm_motor.set_zero_position(0.0);

	// this line records the inertial sensor drifting between initialization and start of auton.
	sys_initial_to_auton_drifting = hardwareParameter.inertialSensorLib.get_rotation();

	/***********************************************
	CHOOSE RUN HERE
	***********************************************/

	// test(); // no slot
	// auton_60s_skills_bridge_version(); // no slot
	// auton_60s_skills_fast_version(); // no slot
	// auton_60s_skills(); // no slot
	// right_side_red(); // slot 1
	// right_side_red_2(); // slot 2
	// right_side_red_3(); // slot 3
	left_side_red(); // slot 4---
	// Red_Double_WP(); // slot 5---
	// right_side_blue(); // slot 6
	// right_side_blue_2(); // slot 7
	// right_side_blue_3(); // slot 8
	// left_side_blue(); // slot 4---
	// Blue_Double_WP(); // slot 5---

	waitForTouch();
}
