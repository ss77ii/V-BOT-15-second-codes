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
	{
		intake_motor.move(-speed);
	}
	else if (mode == OFF)
	{
		intake_motor.move(0);
	}
}

/***********************************************************************************************

AUTONOMOUS PROGRAMS

***********************************************************************************************/

void right_side_red()
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(105.5, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(100, 110, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	if ((pros::millis() - startingtime) <= 3000)
	{ // drop current goal whether or not you have it. go for middle goal
		armAction_1 = {127, 0, -50, 1};
		delay(200);
		turnDegreesPID_lib(350, ON_SPOT_TURN, 127, CLOCKWISE, 2, 0, 0, 1200, 1, hardwareParameter);
		clawAction_1 = {0, false, 1};
		delay(300);
		goStraightCmPID_lib(5, 350, 80, MOVE_BACKWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		turnDegreesPID_lib(115, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		clawAction_1 = {0, true, 1};
		goStraightCmPID_lib(30, 120, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		delay(100);
		// closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);

		// if (closest_goal.width >= 55){
		clawAction_1 = {400, false, 1};
		goStraightCm_Front_Vision(40, 120, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		double angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(25, angle, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		clawAction_1 = {0, true, 1};
		delay(100);
		goStraightCmPID_lib(70, 125, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
		armAction_1 = {127, 0, -50, 1};
		delay(100);
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		goStraightCm_Back_Vision(100, 180, 100, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1800, 1, hardwareParameter);
		delay(200);
		armAction_1 = {125, 0, -200, 1};
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(55, 180, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		hookAction_1 = {0, false, 1};
	}
	//	else if ((pros::millis() - startingtime) < 3000 && (pros::millis() - startingtime) <= 6000){ // go for win point or mid depending on qual or final

	//	}
	else
	{ // keep pulling
		turnDegreesPID_lib(210, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 10000, 1, hardwareParameter);
		armAction_1 = {127, 0, -100, 1};
		goStraightCm_Back_Vision(40, 225, 80, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(25, 225, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	}

	waitForTouch();
}

void right_side_blue()
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(105.5, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(100, 110, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	if ((pros::millis() - startingtime) <= 3000)
	{ // drop current goal whether or not you have it. go for middle goal
		armAction_1 = {127, 0, -50, 1};
		delay(200);
		turnDegreesPID_lib(350, ON_SPOT_TURN, 127, CLOCKWISE, 2, 0, 0, 1200, 1, hardwareParameter);
		clawAction_1 = {0, false, 1};
		delay(300);
		goStraightCmPID_lib(5, 350, 80, MOVE_BACKWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		turnDegreesPID_lib(115, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		clawAction_1 = {0, true, 1};
		goStraightCmPID_lib(30, 120, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		delay(100);
		// closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);

		// if (closest_goal.width >= 55){
		clawAction_1 = {400, false, 1};
		goStraightCm_Front_Vision(40, 120, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		double angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(25, angle, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		clawAction_1 = {0, true, 1};
		delay(100);
		goStraightCmPID_lib(70, 125, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
		armAction_1 = {127, 0, -50, 1};
		delay(100);
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		goStraightCm_Back_Vision(100, 180, 100, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 2000, 1, hardwareParameter);
		delay(200);
		armAction_1 = {125, 0, -200, 1};
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(60, 200, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		hookAction_1 = {0, false, 1};
	}
	//	else if ((pros::millis() - startingtime) < 3000 && (pros::millis() - startingtime) <= 6000){ // go for win point or mid depending on qual or final

	//	}
	else
	{ // keep pulling
		turnDegreesPID_lib(210, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 10000, 1, hardwareParameter);
		armAction_1 = {127, 0, -100, 1};
		goStraightCm_Back_Vision(40, 225, 80, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(25, 225, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	}

	waitForTouch();
}

void left_side_red()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 75;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(112, 75, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(119, 80, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	//	if ((pros::millis() - startingtime) <= 3500){ // drop current goal whether or not you have it. go for middle goal

	armAction_1 = {127, 0, -100, 1};
	delay(100);
	turnDegreesPID_lib(190, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 7, 0, 0, 10000, 2, hardwareParameter);
	goStraightCmPID_lib(10, 190, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1000, 1, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	clawAction_1 = {0, false, 1};
	delay(300);
	goStraightCm_Back_Vision(30, 160, 70, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
	delay(500);
	hookAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 0, -200, 1};
	intakeAction_1 = {100, 500, 0, 100, 1};
	goStraightCmPID_lib(180, 40, 60, MOVE_FORWARD, 4.5, 0, 1, 0.5, 0, 0, 4000, 1, hardwareParameter);
	delay(500);
	goStraightCmPID_lib(20, 40, 100, MOVE_BACKWARD, 4.5, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	waitForTouch();
}

void left_side_blue()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 75;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(112, 75, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(119, 80, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	//	if ((pros::millis() - startingtime) <= 3500){ // drop current goal whether or not you have it. go for middle goal

	armAction_1 = {127, 0, -100, 1};
	delay(100);
	turnDegreesPID_lib(190, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 7, 0, 0, 10000, 2, hardwareParameter);
	goStraightCmPID_lib(10, 190, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1000, 1, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	clawAction_1 = {0, false, 1};
	delay(300);
	goStraightCm_Back_Vision(35, 160, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
	delay(500);
	hookAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 0, -200, 1};
	intakeAction_1 = {100, 500, 0, 100, 1};
	goStraightCmPID_lib(180, 40, 60, MOVE_FORWARD, 4.5, 0, 1, 0.5, 0, 0, 4000, 1, hardwareParameter);
	delay(500);
	goStraightCmPID_lib(20, 40, 100, MOVE_BACKWARD, 4.5, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	waitForTouch();
}

void mid_goal_red()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 119;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(141, 119, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(82, 125, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	armAction_1 = {127, 0, -100, 1};
	delay(100);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 7, 0, 0, 10000, 2, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
	goStraightCm_Back_Vision(76, 119, 90, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.8, 0, 5, 0.7, 0, 1, 1500, 1, hardwareParameter);
	delay(500);
	hookAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 0, -200, 1};
	intakeAction_1 = {100, 500, 0, 100, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 90, CLOCKWISE, 7, 0, 0, 2000, 2, hardwareParameter);
	armAction_1 = {127, 0, -200, 1};
	goStraightCmPID_lib(90, 90, 80, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 2000, 2, hardwareParameter);
	goStraightCmPID_lib(100, 90, 100, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 2000, 2, hardwareParameter);
}

void mid_goal_blue()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 119;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(141, 119, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(82, 125, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	armAction_1 = {127, 0, -100, 1};
	delay(100);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 7, 0, 0, 10000, 2, hardwareParameter);
	goStraightCm_Back_Vision(76, 119, 90, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.8, 0, 5, 0.7, 0, 1, 1500, 1, hardwareParameter);
	delay(500);
	hookAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 0, -200, 1};
	intakeAction_1 = {100, 500, 0, 100, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 90, CLOCKWISE, 7, 0, 0, 2000, 2, hardwareParameter);
	armAction_1 = {127, 0, -200, 1};
	goStraightCmPID_lib(90, 90, 80, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 2000, 2, hardwareParameter);
	goStraightCmPID_lib(100, 90, 100, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 2000, 2, hardwareParameter);
}

void winpoint()
{
	sys_initial_robot_heading = 225;

	hookAction_1 = {0, true, 1};
	delay(200);
	hookAction_1 = {0, false, 1};
	goStraightCmPID_lib(75, 225, 70, MOVE_BACKWARD, 2, 0, 2, 0.5, 0, 5, 1500, 2, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(75, 225, 80, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, -200, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 90, CLOCKWISE, 7, 0, 0, 1500, 3, hardwareParameter);
	intakeAction_1 = {90, 0, 0, 90, 1};
	delay(1000);
	armAction_1 = {127, 0, 0, 1};
	goStraightCm_Front_Vision(70, 90, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	clawAction_1 = {600, true, 1};
	goStraightCmPID_lib(40, angle, 60, MOVE_FORWARD, 2, 0, 2, 1, 0, 5, 1300, 2, hardwareParameter);
	delay(100);
	goStraightCmPID_lib(110, 90, 100, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
}

void finals_left_red()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 75;

	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -400, 1};
	goStraightCmPID_lib(120, 75, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 6, 2000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(2000);
	armAction_1 = {127, 200, 0, 1};
	goStraightCmPID_lib(55, 75, 100, MOVE_BACKWARD, 2, 0, 0.6, 0.65, 0, 4, 1500, 1, hardwareParameter);
	delay(200);
	goStraightCm_Front_Vision(20, 75, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	goStraightCmPID_lib(130, 80, 127, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	goStraightCm_Back_Vision(40, 180, 70, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 0, -600, 1};
	intakeAction_1 = {100, 500, 0, 100, 1};
	goStraightCmPID_lib(180, 40, 60, MOVE_FORWARD, 2.5, 0, 1, 0.5, 0, 0, 4000, 1, hardwareParameter);
	delay(500);
	goStraightCmPID_lib(20, 40, 100, MOVE_BACKWARD, 4.5, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
}

void finals_left_blue()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 75;

	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -400, 1};
	goStraightCmPID_lib(120, 75, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 6, 2000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(2000);
	armAction_1 = {127, 200, 0, 1};
	goStraightCmPID_lib(55, 75, 100, MOVE_BACKWARD, 2, 0, 0.6, 0.65, 0, 4, 1500, 1, hardwareParameter);
	delay(200);
	goStraightCm_Front_Vision(20, 75, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	goStraightCmPID_lib(130, 80, 127, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	goStraightCm_Back_Vision(40, 180, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(200);
	armAction_1 = {127, 0, -600, 1};
	intakeAction_1 = {100, 500, 0, 100, 1};
	goStraightCmPID_lib(180, 40, 60, MOVE_FORWARD, 2.5, 0, 1, 0.5, 0, 0, 4000, 1, hardwareParameter);
	delay(500);
	goStraightCmPID_lib(20, 40, 100, MOVE_BACKWARD, 4.5, 0, 1, 0.5, 0, 0, 800, 1, hardwareParameter);
	armAction_1 = {127, 0, 0, 1};
}

void finals_right_red()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 90;

	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -400, 1};
	goStraightCmPID_lib(112, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 6, 2000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(2000);
	armAction_1 = {127, 200, 0, 1};
	goStraightCmPID_lib(55, 90, 100, MOVE_BACKWARD, 2, 0, 0.6, 0.65, 0, 4, 1500, 1, hardwareParameter);
	delay(200);
	goStraightCm_Front_Vision(20, 90, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	goStraightCmPID_lib(75, 90, 127, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);
	delay(100);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	goStraightCm_Back_Vision(60, 180, 70, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(12, 180, 127, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, -300, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 90, CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	intakeAction_1 = {90, 0, 0, 90, 1};
	goStraightCmPID_lib(90, 90, 75, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	delay(250);
	goStraightCmPID_lib(110, 90, 100, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	hookAction_1 = {0, false, 1};
}

void finals_right_blue()
{
	std::cout << "hi";
	double startingtime = pros::millis();
	sys_initial_robot_heading = 90;

	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -400, 1};
	goStraightCmPID_lib(112, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 6, 2000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(2000);
	armAction_1 = {127, 200, 0, 1};
	goStraightCmPID_lib(55, 90, 100, MOVE_BACKWARD, 2, 0, 0.6, 0.65, 0, 4, 1500, 1, hardwareParameter);
	delay(200);
	goStraightCm_Front_Vision(20, 90, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	goStraightCmPID_lib(75, 90, 127, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);
	delay(100);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	goStraightCm_Back_Vision(60, 180, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(12, 180, 127, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, -300, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 90, CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	intakeAction_1 = {90, 0, 0, 90, 1};
	goStraightCmPID_lib(90, 90, 75, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	delay(250);
	goStraightCmPID_lib(110, 90, 100, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	hookAction_1 = {0, false, 1};
}

void finals_mid_red()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 119;

	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -400, 1};
	goStraightCmPID_lib(143, 119, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 6, 2000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(2000);
	armAction_1 = {127, 200, 0, 1};
	goStraightCmPID_lib(55, 119, 100, MOVE_BACKWARD, 2, 0, 0.6, 0.65, 0, 4, 1500, 1, hardwareParameter);
	delay(200);
	goStraightCm_Front_Vision(20, 119, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	goStraightCmPID_lib(105, 119, 127, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	delay(100);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	goStraightCm_Back_Vision(100, 180, 70, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(12, 180, 127, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, -300, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 90, CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	intakeAction_1 = {90, 0, 0, 90, 1};
	goStraightCmPID_lib(90, 90, 75, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	delay(250);
	goStraightCmPID_lib(110, 90, 100, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	hookAction_1 = {0, false, 1};
}

void finals_mid_blue()
{
	double startingtime = pros::millis();
	sys_initial_robot_heading = 119;

	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -400, 1};
	goStraightCmPID_lib(143, 119, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 6, 2000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(2000);
	armAction_1 = {127, 200, 0, 1};
	goStraightCmPID_lib(55, 119, 100, MOVE_BACKWARD, 2, 0, 0.6, 0.65, 0, 4, 1500, 1, hardwareParameter);
	delay(200);
	goStraightCm_Front_Vision(20, 119, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	double angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	goStraightCmPID_lib(105, 119, 127, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	delay(100);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 90, COUNTER_CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	goStraightCm_Back_Vision(100, 180, 70, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(12, 180, 127, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	armAction_1 = {127, 0, -300, 1};
	turnDegreesPID_lib(90, ON_SPOT_TURN, 90, CLOCKWISE, 6, 0, 0, 1500, 2, hardwareParameter);
	intakeAction_1 = {90, 0, 0, 90, 1};
	goStraightCmPID_lib(90, 90, 75, MOVE_FORWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	delay(250);
	goStraightCmPID_lib(110, 90, 100, MOVE_BACKWARD, 1, 0, 2, 1, 0, 5, 1500, 2, hardwareParameter);
	hookAction_1 = {0, false, 1};
}

void bridgeproof_red()
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(105.5, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(90, 120, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	if ((pros::millis() - startingtime) <= 3000)
	{ // drop current goal whether or not you have it. go for middle goal
		armAction_1 = {127, 0, -350, 1};
		delay(500);
		turnDegreesPID_lib(300, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, 0, 6000, 2, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		delay(300);
		clawAction_1 = {0, false, 1};
		delay(100);
		goStraightCmPID_lib(5, 300, 80, MOVE_BACKWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		turnDegreesPID_lib(135, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 2, hardwareParameter);
		// closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);

		// if (closest_goal.width >= 55){
		clawAction_1 = {400, false, 1};
		goStraightCm_Front_Vision(40, 120, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		double angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(30, angle, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		clawAction_1 = {0, true, 1};
		delay(100);
		goStraightCmPID_lib(70, 125, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
		armAction_1 = {127, 0, -50, 1};
		delay(100);
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		goStraightCm_Back_Vision(110, 180, 100, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1800, 1, hardwareParameter);
		delay(200);
		armAction_1 = {125, 0, -200, 1};
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(25, 180, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	}
	//	else if ((pros::millis() - startingtime) < 3000 && (pros::millis() - startingtime) <= 6000){ // go for win point or mid depending on qual or final

	//	}
	else
	{ // keep pulling
		turnDegreesPID_lib(210, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 10000, 1, hardwareParameter);
		armAction_1 = {127, 0, -100, 1};
		goStraightCm_Back_Vision(40, 225, 80, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(25, 225, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	}

	waitForTouch();
}

void bridgeproof_blue()
{
	sys_initial_robot_heading = 90;
	double startingtime = pros::millis();
	vision_object_s_t closest_goal;

	hookAction_1 = {0, true, 1};
	goStraightCmPID_lib(105.5, 90, 127, MOVE_FORWARD, 2, 0, 0.6, 0.65, 0, 4, 5000, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	hookAction_1 = {0, false, 1};
	pros::lcd::print(0, "time at grab: %f", pros::millis() - startingtime);
	delay(100);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(90, 120, 127, MOVE_BACKWARD, 2, 0, 2, 1, 0, 5, 15000, 2, hardwareParameter);
	pros::lcd::print(1, "time after: %f", pros::millis() - startingtime);

	if ((pros::millis() - startingtime) <= 3000)
	{ // drop current goal whether or not you have it. go for middle goal
		armAction_1 = {127, 0, -350, 1};
		delay(500);
		turnDegreesPID_lib(300, ON_SPOT_TURN, 127, CLOCKWISE, 6, 0, 0, 6000, 2, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		delay(300);
		clawAction_1 = {0, false, 1};
		delay(100);
		goStraightCmPID_lib(5, 300, 80, MOVE_BACKWARD, 2, 0, 2, 10, 0, 2, 600, 1, hardwareParameter);
		armAction_1 = {127, 0, 0, 1};
		turnDegreesPID_lib(135, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 2, hardwareParameter);

		clawAction_1 = {400, false, 1};
		goStraightCm_Front_Vision(40, 120, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
		double angle = get_robot_heading_lib(hardwareParameter);
		goStraightCmPID_lib(30, angle, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
		clawAction_1 = {0, true, 1};
		delay(100);
		goStraightCmPID_lib(70, 125, 127, MOVE_BACKWARD, 2, 0, 1, 0.5, 0, 0, 1200, 1, hardwareParameter);
		armAction_1 = {127, 0, -50, 1};
		delay(100);
		turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.6, 0, -1, 1200, 2, hardwareParameter);
		goStraightCm_Back_Vision(80, 180, 80, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1800, 1, hardwareParameter);
		delay(200);
		armAction_1 = {125, 0, -200, 1};
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(25, 180, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	}

	else
	{ // keep pulling
		turnDegreesPID_lib(210, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 6, 0, 0, 10000, 1, hardwareParameter);
		armAction_1 = {127, 0, -100, 1};
		goStraightCm_Back_Vision(40, 225, 80, DETECT_BLUE_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
		hookAction_1 = {0, true, 1};
		delay(400);
		intakeAction_1 = {100, 0, 0, 100, 1};
		delay(500);
		goStraightCmPID_lib(25, 225, 100, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 0, 1500, 1, hardwareParameter);
	}

	waitForTouch();
}

void auton_60s_skills()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long start_time = pros::millis();

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, true, 1};
	arm_motor.move(60);
	delay(150);
	hookAction_1 = {0, false, 1};
	delay(200);
	// ------------------------------------------------------------------------------------ //
	//// part 1 take first blue goal
	goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(100);

	goStraightCmPID_lib(80, 80, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 2500, 1, hardwareParameter); // 100,80,127
	arm_motor.move(0);
	arm_motor.tare_position();

	goStraightCm_Front_Vision(25, 80, 100, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.5, 0, 7, 1, 0, 1, 700, 1, hardwareParameter);
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 70, MOVE_FORWARD, 1.5, 0, 1.5, 1, 0, 0, 900, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 5, RELEASE_BRIDGE - 100, 1};
	delay(100);
	intakeAction_1 = {90, 400, 0, 90, 1};
	delay(500);
	goStraightCmPID_lib(150, 62, 95, MOVE_FORWARD, 3.5, 0, 2.5, 0.2, 0, 5, 1650, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 25, 1};
	delay(250);
	goStraightCmPID_lib(15, 62, 127, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 5, 500, 1, hardwareParameter);
	clawAction_1 = {0, false, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};

	////place first yellow goal
	//	delay(300);
	////take first blue goal by front claw
	goStraightCmPID_lib(5, 63, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 300, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 100, 1};
	delay(150);
	goStraightCmPID_lib(70, 55, 127, MOVE_BACKWARD, 5, 0, 2.5, 0.5, 0, 5, 1200, 1, hardwareParameter);

	//	intakeAction_1 = {0, 0, 0, 0, 1};
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(100);
	goStraightCmPID_lib(27, 55, 100, MOVE_FORWARD, 5, 0, 1, 0.5, 0, 10, 700, 1, hardwareParameter);
	turnDegreesPID_lib(235, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter);
	goStraightCmPID_lib(27, 235, 60, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 700, 1, hardwareParameter);

	//  goStraightCm_Front_Vision(30, 239, 100, DETECT_BLUE_GOAL_SIG, front_vision,
	//	  									0.5, 0, 1, 0.3, 0, 7, 0.3, 0, 1, 1000, 1, hardwareParameter);

	delay(100);
	clawAction_1 = {0, true, 1};
	armAction_1 = {127, 0, -250, 1};
	delay(100);
	goStraightCmPID_lib(75, 350, 127, MOVE_BACKWARD, 1.5, 0, 1.2, 0.7, 0, 10, 1500, 1, hardwareParameter);
	goStraightCm_Back_Vision(16, 350, 80, DETECT_RED_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 400, 1, hardwareParameter);
	////take first red goal by back claw
	//  delay(100);
	hookAction_1 = {0, true, 1};
	armAction_1 = {127, 5, PRESS_BRIDGE - 300, 1};
	intakeAction_1 = {90, 00, 0, 90, 1};
	// waitForTouch();
	goStraightCmPID_lib(102, 347, 90, MOVE_FORWARD, 3.5, 0, 2.5, 1, 0, 0, 2000, 1, hardwareParameter);
	goStraightCmPID_lib(90, 0, 75, MOVE_FORWARD, 3.5, 0, 2.5, 0.5, 0, 0, 2500, 2, hardwareParameter);
	//	delay(100);
	//	goStraightCmPID_lib(60, 0, 100, MOVE_BACKWARD, 1.5, 0, 1, 0.5, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	turnDegreesPID_lib(105, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter);

	intakeAction_1 = {90, 00, 0, 90, 1};
	goStraightCmPID_lib(50, 105, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	delay(100);
	goStraightCmPID_lib(5, 105, 100, MOVE_FORWARD, 1.5, 0, 1, 1, 0, 5, 300, 1, hardwareParameter);
	////place blue mobile goal
	clawAction_1 = {0, false, 1};
	delay(100);
	goStraightCmPID_lib(5, 105, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 150, 1};
	delay(200);

	////take middle high yellow goal
	goStraightCmPID_lib(12, 180, 100, MOVE_BACKWARD, 1.5, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	turnDegreesPID_lib(230, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter);
	armAction_1 = {127, 0, 20, 1};
	delay(600);
	goStraightCm_Front_Vision(30, 230, 70, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1000, 1, hardwareParameter);
	intakeAction_1 = {-10, 0, 0, -10, 1};
	angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, angle, 45, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 1000, 1, hardwareParameter);
	delay(100);
	clawAction_1 = {0, true, 1};
	// waitForTouch();
	delay(100);
	armAction_1 = {127, 0, PRESS_BRIDGE - 300, 1};
	delay(400);
	intakeAction_1 = {90, 00, 0, 90, 1};
	// move high yellow goal to red bridge
	angle = 276;
	goStraightCmPID_lib(120, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 2500, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE + 10, 1};
	goStraightCmPID_lib(5, angle, 70, MOVE_FORWARD, 1.5, 0, 0, 1, 0, 5, 350, 1, hardwareParameter);
	delay(400);
	////place high yellow mobile goal
	clawAction_1 = {0, false, 1};
	delay(100);
	goStraightCmPID_lib(5, angle, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 350, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 100, 1};
	delay(200);
	goStraightCmPID_lib(45, 270, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	intakeAction_1 = {0, 0, 0, 0, 1};
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	delay(100);
	goStraightCmPID_lib(20, 270, 100, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 10, 800, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 100, COUNTER_CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter); // shit
	goStraightCmPID_lib(40, 90, 55, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 1200, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(100);

	armAction_1 = {127, 0, PRESS_BRIDGE - 300, 1};
	delay(500);

	angle = 250;
	turnDegreesPID_lib(angle, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 1000, 2, hardwareParameter);
	goStraightCmPID_lib(95, angle, 100, MOVE_FORWARD, 3, 0, 1, 0.5, 0, 8, 1750, 1, hardwareParameter);
	armAction_1 = {127, 300, PRESS_BRIDGE, 1};
	turnDegreesPID_lib(275, ON_SPOT_TURN, 80, COUNTER_CLOCKWISE, 1.6, 0, -1, 800, 1, hardwareParameter);
	delay(250);
	////place red mobile goal in red bridge
	clawAction_1 = {0, false, 1};
	delay(100);
	goStraightCmPID_lib(5, 270, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 100, 1};
	delay(200);
	goStraightCmPID_lib(15, angle + 10, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 600, 1, hardwareParameter);
	//	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0, 1200, 3, hardwareParameter); 				//shit
	turnDegreesPID_lib(180, ON_SPOT_TURN, 100, CLOCKWISE, 6, 0, 0, 1000, 3, hardwareParameter); // shit
	armAction_1 = {127, 0, 0, 1};
	goStraightCmPID_lib(120, 182, 100, MOVE_BACKWARD, 2, 0, 1, 1, 0, 5, 1800, 1, hardwareParameter);
	goStraightCm_Back_Vision(40, 180, 60, DETECT_BLUE_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1000, 1, hardwareParameter);
	// take corner blue goal
	hookAction_1 = {0, true, 1};
	delay(100);

	goStraightCmPID_lib(50, 180, 90, MOVE_FORWARD, 2, 0, 1, 0.5, 0, 5, 900, 1, hardwareParameter);
	turnDegreesPID_lib(90, ON_SPOT_TURN, 80, CLOCKWISE, 3, 0, 0, 900, 3, hardwareParameter);
	goStraightCm_Front_Vision(30, 90, 90, DETECT_YELLOW_GOAL_SIG, front_vision,
							  0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 800, 1, hardwareParameter);
	// angle = get_robot_heading_lib(hardwareParameter);
	goStraightCmPID_lib(30, 90, 50, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 0, 800, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, PRESS_BRIDGE - 300, 1};
	intakeAction_1 = {90, 300, 0, 90, 1};
	goStraightCmPID_lib(135, 120, 100, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 2000, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	delay(200);
	goStraightCmPID_lib(10, 135, 100, MOVE_FORWARD, 3.5, 0, 2.5, 0.4, 0, 3, 400, 1, hardwareParameter);
	////////////////push left
	//	angle = get_robot_heading_lib(hardwareParameter);
	//	turnDegreesPID_lib(angle + 20, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 5, 0, 0, 500, 1, hardwareParameter);
	// release third yellow goal
	clawAction_1 = {0, false, 1};
	delay(100);
	goStraightCmPID_lib(5, 120, 100, MOVE_BACKWARD, 3.5, 0, 2.5, 3, 0, 5, 400, 1, hardwareParameter);
	armAction_1 = {127, 0, PRESS_BRIDGE - 100, 1};
	delay(200);
	goStraightCmPID_lib(55, 120, 100, MOVE_BACKWARD, 3, 0, 1, 1, 0, 5, 1000, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	armAction_1 = {127, 0, 0, 1};
	intakeAction_1 = {0, 0, 0, 0, 1};
	delay(100);
	goStraightCmPID_lib(25, 120, 100, MOVE_FORWARD, 5, 0, 1, 1, 0, 10, 1000, 1, hardwareParameter);
	turnDegreesPID_lib(300, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1200, 1, hardwareParameter);
	goStraightCmPID_lib(40, 300, 55, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 1500, 1, hardwareParameter);
	delay(100);
	clawAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, PRESS_BRIDGE - 300, 1};
	delay(400);
	turnDegreesPID_lib(115, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, -1, 1200, 2, hardwareParameter);
	goStraightCmPID_lib(80, 115, 80, MOVE_FORWARD, 3, 0, 1, 0.3, 0, 0, 1500, 1, hardwareParameter);
	// release last blue goal
	clawAction_1 = {0, false, 1};
	delay(100);

	armAction_1 = {127, 300, -400, 1};
	goStraightCmPID_lib(20, 20, 100, MOVE_BACKWARD, 5, 0, 1, 0.3, 0, 0, 800, 1, hardwareParameter);
	delay(400);
	goStraightCmPID_lib(104, 20, 90, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 8, 1700, 1, hardwareParameter);
	//	goStraightCmPID_lib(15, 90, 90, MOVE_BACKWARD,	5, 0, 1, 0.3, 0, 8, 500, 1, hardwareParameter);
	armAction_1 = {127, 0, -300, 1};
	turnDegreesPID_lib(315, ON_SPOT_TURN, 100, CLOCKWISE, 1.2, 0, -1, 800, 2, hardwareParameter);

	goStraightCm_Back_Vision(47, 315, 60, DETECT_RED_GOAL_SIG, back_vision,
							 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1200, 1, hardwareParameter);
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(100);
	turnDegreesPID_lib(290, ON_SPOT_TURN, 80, CLOCKWISE, 1.2, 0, -1, 800, 2, hardwareParameter);
	intakeAction_1 = {90, 00, 0, 90, 1};

	goStraightCmPID_lib(100, 285, 75, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 8, 2000, 1, hardwareParameter);
	if (pros::millis() - start_time >= 5700)
	{
		hookAction_1 = {1600, false, 1};
		goStraightCmPID_lib(200, 280, 110, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 8, 3000, 1, hardwareParameter);
	}
	else
	{
		goStraightCmPID_lib(200, 280, 110, MOVE_FORWARD, 5, 0, 1, 0.3, 0, 8, 3000, 1, hardwareParameter);
		hookAction_1 = {0, false, 1};
	}

	pros::lcd::print(2, "Time=%d", pros::millis() - start_time);
	waitForTouch();

	//	pros::lcd::print(1, "time=%d", pros::millis() - startTime);
	waitForTouch();
	//*****************************************************
}

void Red_Double_WP()
{
	sys_initial_to_auton_drifting = inertial_sensor.get_rotation();
	sys_initial_robot_heading = 180;
	long start_time = pros::millis();

	clawAction_1 = {0, false, 1};
	hookAction_1 = {0, true, 1};
	arm_motor.move(60);
	delay(150);
	hookAction_1 = {0, false, 1};
	delay(200);

	// ------------------------------------------------------------------------------------ //
	// part 1, take the first red goal
	goStraightCmPID_lib(5.5, 180, 80, MOVE_BACKWARD, 0, 0, 0, 5, 0, 0, 500, 1, hardwareParameter);
	delay(100);
	hookAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};

	goStraightCmPID_lib(90, 80, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 2500, 1, hardwareParameter); // 100,80,127
	delay(200);
	turnDegreesPID_lib(0, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, -1, 1000, 1, hardwareParameter);
	intakeAction_1 = {127, 0, 0, 127, 2};
	intakeAction_2 = {0, 2300, 0, 0, 2};
	goStraightCmPID_lib(20, 0, 100, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 700, 1, hardwareParameter);
	goStraightCmPID_lib(130, 0, 45, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0, 3000, 1, hardwareParameter);
	turnDegreesPID_lib(135, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, 0, 1200, 1, hardwareParameter);
	armAction_1 = {127, 0, 10, 1};
	goStraightCmPID_lib(70, 135, 127, MOVE_BACKWARD, 2.0, 0, 2.5, 0.4, 0, 3, 1800, 1, hardwareParameter);
	hookAction_1 = {0, false, 1};
	delay(100);
	intakeAction_1 = {0, 0, 0, 0, 1};

	// part 2

	goStraightCmPID_lib(20, 135, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 800, 1, hardwareParameter);
	turnDegreesPID_lib(180, ON_SPOT_TURN, 127, COUNTER_CLOCKWISE, 1.2, 0, 0, 700, 1, hardwareParameter);
	goStraightCm_Back_Vision(50, 180, 70, DETECT_RED_GOAL_SIG, back_vision, 0.5, 0, 1, 0.5, 0, 5, 1, 0, 1, 1500, 1, hardwareParameter);
	hookAction_1 = {0, true, 1};
	delay(100);
	intakeAction_1 = {127, 200, 0, 127, 1};
	goStraightCmPID_lib(35, 180, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 1500, 1, hardwareParameter);
	turnDegreesPID_lib(135, ON_SPOT_TURN, 127, CLOCKWISE, 1.2, 0, 0, 1500, 1, hardwareParameter);
	goStraightCmPID_lib(45, 135, 127, MOVE_FORWARD, 2.0, 0, 2.5, 1, 0, 0.001, 1500, 1, hardwareParameter);
	goStraightCm_Front_Vision(75, 135, 90, DETECT_YELLOW_GOAL_SIG, front_vision, 0.5, 0, 1, 0.3, 0, 10, 0.4, 0, 0, 1500, 1, hardwareParameter);
	clawAction_1 = {0, true, 1};
	delay(100);
	armAction_1 = {127, 0, PRESS_BRIDGE, 1};
	goStraightCmPID_lib(70, 135, 127, MOVE_BACKWARD, 2.0, 0, 2.5, 1, 0, 0, 1500, 1, hardwareParameter);
}

void Blue_Double_WP()
{
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

	// right_side_red();
	// right_side_blue();
	// left_side_red();
	// left_side_blue();
	// mid_goal_red();
	// mid_goal_blue();
	// auton_60s_skills();
	// winpoint();
	// finals_left_red();
	// finals_left_blue();
	// finals_right_red();
	// finals_right_blue();
	// finals_mid_red(); // DOESNT WORK
	// finals_mid_blue(); // DOESNT WORK
	// bridgeproof_red();
	// bridgeproof_blue();
	Red_Double_WP();

	waitForTouch();
}
