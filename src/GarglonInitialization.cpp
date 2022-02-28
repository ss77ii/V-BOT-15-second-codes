#include "../include/main.h"
#include "GarglonConfiguration.hpp"

using namespace pros;

int function_selection = SELECTION_DRIVE;
bool is_starting_selection_confirmed = false;
int auto_15_start_selection;

pros::Task intake_set(intake_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake set");
pros::Task hook_set(hook_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "hook set");
pros::Task claw_set(claw_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "claw set");
pros::Task arm_set(arm_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm set");

void display_vision_error(int line, std::string msg){
	if(errno == 0){
		pros::lcd::print(line, "n=%d  %s", errno, msg);
	} else {
		std::string str = "";
		if(errno == ENODEV){
			str = "port not vision";
		} else if(errno == EINVAL){
			str = "sig_id out of range";
		} else if(errno == EDOM) {
			str = "size_id too large";
		} else if(errno == EAGAIN){
			str = "read vision fail";
		} else {
			str = "unknown";
		}
		pros::lcd::print(line, "n=%d  %s", errno, str);
    waitForTouch();
	}
}

void initialize()
{
	arm_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_front_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_back_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right_mid_motor.set_brake_mode(E_MOTOR_BRAKE_COAST);

	arm_motor.tare_position();
	left_front_motor.tare_position();
	left_back_motor.tare_position();
	left_mid_motor.tare_position();
	right_front_motor.tare_position();
	right_back_motor.tare_position();
	right_mid_motor.tare_position();
	intake_motor.tare_position();

	arm_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	left_front_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	left_back_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	left_mid_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	right_front_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	right_back_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	right_mid_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	intake_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	Y_encoder.reset();

	// pros::vision_signature_s_t red_goal_sig = pros::Vision::signature_from_utility(
	//  	                    DETECT_RED_GOAL_SIG, 7527, 9761, 8644, -1293, -819, -1056, 2.400, 0);
 //
	// pros::vision_signature_s_t yellow_goal_sig = pros::Vision::signature_from_utility(
	//  										  DETECT_YELLOW_GOAL_SIG, 2127, 2595, 2360, -3867, -3397, -3632, 3.500, 0);
 //
 // pros::vision_signature_s_t blue_goal_sig = pros::Vision::signature_from_utility(
 // 										  DETECT_BLUE_GOAL_SIG, -1917, -1095, -1506, 5259, 8681, 6970, 2.200, 0);
 //
	pros::vision_signature_s_t red_goal_sig = pros::Vision::signature_from_utility(
		                    DETECT_RED_GOAL_SIG, 7761, 9095, 8428, -907, -503, -705, 4.400, 0);
	pros::vision_signature_s_t blue_goal_sig = pros::Vision::signature_from_utility(
											  DETECT_BLUE_GOAL_SIG, -2383, -1413, -1898, 8113, 11627, 9870, 3.100, 0);
	pros::vision_signature_s_t yellow_goal_sig = pros::Vision::signature_from_utility(
											  DETECT_YELLOW_GOAL_SIG, 1783, 2259, 2021, -3573, -3215, -3394, 5.500, 0);

	front_vision.clear_led();
	front_vision.set_wifi_mode(0); //disable wifi
  front_vision.set_signature(DETECT_RED_GOAL_SIG, &red_goal_sig);
  front_vision.set_signature(DETECT_BLUE_GOAL_SIG, &blue_goal_sig);
	front_vision.set_signature(DETECT_YELLOW_GOAL_SIG, &yellow_goal_sig);

	back_vision.clear_led();
	back_vision.set_wifi_mode(0); //disable wifi
  back_vision.set_signature(DETECT_RED_GOAL_SIG, &red_goal_sig);
  back_vision.set_signature(DETECT_BLUE_GOAL_SIG, &blue_goal_sig);
	back_vision.set_signature(DETECT_YELLOW_GOAL_SIG, &yellow_goal_sig);

	pros::lcd::initialize();
	inertial_sensor.reset();
  delay(2500);

//	pros::lcd::print(0, "aaaaaaaaaaaaaaaa");
//	std::cout<<"bbbbbbbbbb"<<std::endl;

/*
display_vision_error(0, "set sig");
while(true){
pros::lcd::print(1, "%d",
			front_vision_sensor.get_by_sig(0, DETECT_BLUE_GOAL_SIG).x_middle_coord);
delay(50);
}
*/

/*
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(lcd_left_button);   //red or blue team selection button
	pros::lcd::register_btn1_cb(lcd_center_button); //square or rectangle selection button
	pros::lcd::register_btn2_cb(lcd_right_button);  //confirm button
	pros::delay(500);
	pros::lcd::set_text(0, "         ----16868A----");
	pros::lcd::set_text(1, "Inertial sensor initializing......");
	inertial_sensor.reset();
	pros::delay(3000);

	pros::lcd::set_text(2, "Inertial initialization completed");
	pros::lcd::set_text(3, "Inertial sensor testing......");
	double temp = pros::millis();
	while(true){ // in the very beginning of the code, the inertial sensor has yet to begin working and its output in infinity, this loop waits until there are real values to operate with before beginning
		double currentHeading = inertial_sensor.get_rotation();
		if (currentHeading != infinity()){
			break;
		}
		else if (pros::millis() >= temp + 5000){
			pros::lcd::set_text(3, "Inertial sensor error.");
			pros::lcd::set_text(4, "*********** Restart required ************");
			waitForTouch();
		}
		pros::delay(5);
	}
	pros::lcd::set_text(1, "Inertial testing successes");
	pros::lcd::clear_line(1);
	pros::lcd::clear_line(2);
	pros::lcd::clear_line(3);
	pros::lcd::clear_line(4);


	display_slection(7);
  pros::lcd::set_text(5, "Please confirm selection");
	while(is_starting_selection_confirmed == false){
		pros::delay(5);
	}
*/

//	function_selection = SELECTION_DRIVE;
//	function_selection = SELECTION_AUTO;

//	auto_15_start_selection = SELECTION_TESTING;
//	auto_15_start_selection = AUTO_15_BLUE_START_I;  //slot 2
//	auto_15_start_selection = AUTO_15_RED_START_G;
//	auto_15_start_selection = AUTO_15_BLUE_START_C;
//	auto_15_start_selection = AUTO_15_RED_START_A;  //slot 1
//  auto_15_start_selection = AUTO_CORNER_15_BLUE_START_I;
//  auto_15_start_selection = AUTO_CORNER_15_RED_START_A;

//  auto_15_start_selection = AUTO_15_ELEVEN_RED_A;   //slot 4
//  auto_15_start_selection = AUTO_15_ELEVEN_BLUE_I;  //slot 5

//	pros::Task intake_set(intake_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake set");
//  pros::Task hook_set(hook_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "hook set");
//  pros::Task claw_set(claw_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "claw set");
//	pros::Task arm_set(arm_set_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "arm set");

//	pros::Task odometry(odometry_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
//	pros::Task infoPrint(infoPrint_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "infoprint");
}


void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
