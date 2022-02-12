#include "../include/main.h"
#include "Garglonmath_lib.hpp"

/****************************************************************************************************************
 * 2022 Game field coodinate system setting:
 *****************************************************************************************************************/

using namespace pros;

#define WHEEL_CIRCUMFERENCE_CM 31.918
#define ENCODER_CIRCUMFERENCE_CM 21.944

#define ENCODER_X_FROM_CENTRE 2.5
#define ENCODER_Y_FROM_CENTRE -4
// MEASURE WHEN THE ROBOT FACES TO THE Opponent ie. 90 degrees
// ENSURE THAT Y DIFFERENCE IS THE VERTICAL AND X DIFFERENCE IS THE HORIZONTAL IN THIS ORIENTATION
// make either negative according to cartesian coordinates

// coordinates of centre of robot
#define ROBOT_INITIAL_X 73.5
#define ROBOT_INITIAL_Y 28.5

#define ROBOT_INITIAL_HEADING 90

#define COODINATE_UPDATE_X 1
#define COODINATE_UPDATE_Y 2
#define COODINATE_UPDATE_XY 3
#define COODINATE_UPDATE_INCREMENT 4

#define SELECTION_AUTO 0
#define SELECTION_DRIVE 1

// port assignments //
#define RIGHT_FRONT_DRIVE_MOTOR_PORT 12
#define RIGHT_MID_DRIVE_MOTOR_PORT 14
#define RIGHT_BACK_DRIVE_MOTOR_PORT 16
#define LEFT_FRONT_DRIVE_MOTOR_PORT 9
#define LEFT_MID_DRIVE_MOTOR_PORT 7
#define LEFT_BACK_DRIVE_MOTOR_PORT 5
#define INTAKE_MOTOR_PORT 18
#define ARM_MOTOR_PORT 20

#define INERTIAL_SENSOR_PORT 1

#define FRONT_VISION_PORT 4
#define BACK_VISION_PORT 17

#define EXT_EXPANDER_PORT 3

#define Y_ENCODER_TOP_PORT 'G'
#define Y_ENCODER_BOTTOM_PORT 'H'

#define PNEUMATIC_BACK_PORT 'F'
#define PNEUMATIC_FRONT_PORT 'B'
#define PNEUMATIC_TOP_PORT 'C'
#define TOUCH_SENSOR_PORT 'D'

#define MOVE_FORWARD 1
#define MOVE_BACKWARD -1
#define MOVE_RIGHT 1
#define MOVE_LEFT -1

#define ONE_WHEEL_TURN_FORWARD 2
#define ONE_WHEEL_TURN_BACKWARD 3
#define TWO_WHEEL_TURN_FORWARD 2
#define TWO_WHEEL_TURN_BACKWARD 3
#define ON_SPOT_TURN 1

#define COUNTER_CLOCKWISE -1
#define CLOCKWISE 1
#define TURN_SMALLEST_ANGLE 0

#define FRONT_VISION_X_MIDDLE 158
#define BACK_VISION_X_MIDDLE 158

#define DETECT_NO_GOAL 0
#define DETECT_RED_GOAL_SIG 1
#define DETECT_BLUE_GOAL_SIG 2
#define DETECT_YELLOW_GOAL_SIG 3

// the following defines are copied from XBotAutonomous_lib.hpp so that the hpp files can be combined
#define STOP_AT_END true
#define ROTATE_AT_END false

/*********************************************************
 * background command
 *********************************************************/
#define AUTON_15_CMD_IDLE 0
#define AUTON_15_CMD_PREPARE_ONE_RED_BALL 1
#define AUTON_15_CMD_PREPARE_ONE_BLUE_BALL 2

/********************************************************
 * Fly Wheel related definitions
 *********************************************************/
#define CMD_CLAW_STOP_ACTION 0
#define CMD_CLAW_ONE_ACTION 1
#define CMD_CLAW_TWO_ACTIONS 2
#define CMD_CLAW_TRI_ACTIONS 3

// claw states
#define CLAW_CATCH -410
#define CLAW_PRE -280
#define CLAW_RELEASE -190

#define CLAW_ACTION_NONE 100
#define CLAW_ACTION_BY_DELAY 101
#define CLAW_ACTION_BY_ENCODER 102

#define CLAW_ROTATE_NONE 200
#define CLAW_ROTATE_UP 201
#define CLAW_ROTATE_DOWN 202

/*********************************************************
 * HOOK related definitions
 *********************************************************/
#define CMD_HOOK_STOP_ACTION 10
#define CMD_HOOK_ONE_ACTION 11
#define CMD_HOOK_TWO_ACTIONS 12
#define CMD_HOOK_TRI_ACTIONS 13

// hook state definitions
#define HOOK_CATCH 195
#define HOOK_PRE 120
#define HOOK_RELEASE 0

#define HOOK_ACTION_NONE 110
#define HOOK_ACTION_BY_DELAY 111
#define HOOK_ACTION_BY_ENCODER 112

#define HOOK_ROTATE_NONE 210
#define HOOK_ROTATE_UP 211
#define HOOK_ROTATE_DOWN 212

/*********************************************************
 * ARM related definitions
 *********************************************************/

#define PRESS_BRIDGE -380
#define RELEASE_BRIDGE -740
#define MAX_TOP -1000
#define CARRY_GOAL -250

/*********************************************************
 * Intake related definitions
 *********************************************************/
#define CMD_INTAKE_STOP_ACTION 20
#define CMD_INTAKE_ONE_ACTION 21
#define CMD_INTAKE_TWO_ACTIONS 22
#define CMD_INTAKE_TRI_ACTIONS 23

#define INTAKE_ACTION_NONE 120
#define INTAKE_ACTION_BY_DELAY 121
#define INTAKE_ACTION_BY_ENCODER 122

#define INTAKE_ROTATE_NONE 220
#define INTAKE_ROTATE_IN 221
#define INTAKE_ROTATE_OUT 222

/************************************************
ARM related definitions
************************************************/
#define CMD_ARM_STOP_ACTION 20
#define CMD_ARM_ONE_ACTION 21
#define CMD_ARM_TWO_ACTIONS 22
#define CMD_ARM_TRI_ACTIONS 23

#define INTAKE_ARM_NONE 120
#define INTAKE_ARM_BY_DELAY 121
#define INTAKE_ARM_BY_ENCODER 122

#define ARM_ROTATE_NONE 220
#define ARM_ROTATE_IN 221
#define ARM_ROTATE_OUT 222

//**********************************************

#define INTAKE_BALL_NONE_CMD 10
#define INTAKE_RED_BALL_RESET 11
#define INTAKE_BLUE_BALL_RESET 12

#define WHEEL_CIRCUMFERENCE 21.9433775

#define RED_HUE_BOUNDARY 30
#define BLUE_HUE_BOUNDARY 160

struct Hardware
{
    pros::Motor leftFrontMotorLib;
    pros::Motor leftBackMotorLib;
    pros::Motor leftMidMotorLib;
    pros::Motor rightFrontMotorLib;
    pros::Motor rightBackMotorLib;
    pros::Motor rightMidMotorLib;
    pros::Motor intakeMotorLib;
    pros::Motor armMotorLib;
    pros::ADIDigitalOut frontPistonLib;
    pros::ADIDigitalOut backPistonLib;
    pros::ADIDigitalOut topPistonLib;
    pros::Imu inertialSensorLib;
    pros::ADIEncoder Y_encoderLib;
};

typedef struct
{
    float leftFrontMotorPower;
    float rightFrontMotorPower;
    float leftBackMotorPower;
    float rightBackMotorPower;
} MotorController;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
} PID_config;

//
typedef struct
{
    bool hold_heading_lib;
    float holding_angle_lib;
} HeadingStruct;

// parallel movements structures

typedef struct
{
    long startingDelay_ms;
    bool actionType; // true = activated, false = deactivated
    int runTimes;
} HookAction;

typedef struct
{
    long startingDelay_ms;
    bool actionType;
    int runTimes;
} ClawAction;

typedef struct
{
    int motorPower;
    long startingDelay_ms;
    int desiredAbsolute;
    int runTimes;
} ArmAction;

typedef struct
{
    int motorPower;
    long startingDelay_ms;
    long lastingTime_ms;
    int endPower;
    int runTimes;
} IntakeAction;

extern int auto_15_start_selection;
extern int function_selection;
// motor declarations //
extern pros::Motor left_front_motor;
extern pros::Motor right_front_motor;
extern pros::Motor left_mid_motor;
extern pros::Motor right_mid_motor;
extern pros::Motor left_back_motor;
extern pros::Motor right_back_motor;
extern pros::Motor intake_motor;
extern pros::Motor arm_motor;
extern pros::ADIDigitalOut back_piston;
extern pros::ADIDigitalOut front_piston;
extern pros::ADIDigitalOut top_piston;

extern pros::Imu inertial_sensor;
extern pros::ADIPort touch_sensor;

extern pros::ADIEncoder Y_encoder;

extern pros::Controller master;

// extern bool door_status;

// This variable defines the drifting angle during the period
// after Inertial sensor reset in Initialization() and before the starting of auton.
extern double sys_initial_to_auton_drifting;
extern double sys_initial_robot_heading;
extern double motion_initial_angle; // used by position tracing system,
                                    // value changed in motion function.

extern Hardware hardwareParameter;
extern Point sys_coordinates;
// extern Point sys_encoderPosition;
extern bool coordinateAccessAllowed;

extern long flyWheelSet;
extern long HOOKSet;
extern long intakesSet;
extern long driveSet;

extern ClawAction clawAction_1;
extern ClawAction clawAction_2;
extern ClawAction clawAction_3;
extern int cmd_flyWheel_run;

extern HookAction hookAction_1;
extern HookAction hookAction_2;
extern HookAction hookAction_3;
extern int cmd_elevator_run;

extern IntakeAction intakeAction_1;
extern IntakeAction intakeAction_2;
extern IntakeAction intakeAction_3;
extern int cmd_intake_run;

extern ArmAction armAction_1;
extern ArmAction armAction_2;
extern ArmAction armAction_3;
extern int cmd_arm_run;
extern bool checkpointBreak;

extern bool clawBreak;
extern bool hookBreak;
extern bool intakeBreak;
extern bool armBreak;

// auton tasks for multitasking
extern void hook_set_fn(void *param);
extern void claw_set_fn(void *param);
extern void intake_set_fn(void *param);
extern void arm_set_fn(void *param);
extern void background_command_fn(void *param);
extern void breakAll();

extern long checkpoint;
extern int auton_15_command;

extern pros::Task intake_set;
extern pros::Task hook_set;
extern pros::Task claw_set;
extern pros::Task arm_set;

extern pros::Vision front_vision;
extern pros::Vision back_vision;
extern pros::vision_object_s_t closest_red_goal;
extern pros::vision_object_s_t closest_blue_goal;
extern pros::vision_object_s_t closest_yellow_goal;

void waitForTouch();

double convert_target_to_relative_angle_lib(double currentRelativeHeading, double targetHeading);

void goStraightCmPID_lib(double cmDistance, double robotHeadingLib, int maxSpeed, int robotDirection, double headingKP, double headingKI, double headingKD, double distanceKP, double distanceKI, double distanceKD, long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void turnDegreesPID_lib(double targetHeadingLib, double turnType, long maxPower, int turnDirection, double headingKP, double headingKI, double headingKD, long timeoutMili, int exitConditionExpectedPasses, Hardware config);

void twoWheelTurnDegreesPID(double targetHeadingLib, double turnType, double smallerPower, double largerPower, int turnDirection, double headingKP,
                            double headingKI, double headingKD, long timeoutMili, int exitConditionExpectedPasses, Hardware config);

double get_robot_heading_radians_lib(Hardware robot);
double get_robot_heading_lib(Hardware robot);

void update_coordinate(Point p);
void update_Coodinate(int updateType, double pos);
void update_coordinate(int updateType, double xPos, double yPos);

Point get_coordinate();

void odometry_fn(void *param);

void infoPrint_fn(void *param);

void goStraightCm_Front_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                               int goal_color_signature, Vision vision_sensor,
                               double headingKP, double headingKI, double headingKD,
                               double distanceKP, double distanceKI, double distanceKD,
                               double visionKP, double visionKI, double visionKD,
                               long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void goStraightCm_Back_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                              int goal_color_signature, Vision vision_sensor,
                              double headingKP, double headingKI, double headingKD,
                              double distanceKP, double distanceKI, double distanceKD,
                              double visionKP, double visionKI, double visionKD,
                              long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void searchGoal_Vision(int maxSpeed, int searchDirection, double maxSearchintAngle,
                       double goal_min_width, int goal_color_signature, pros::Vision vision_sensor,
                       double visionKP, double visionKI, double visionKD,
                       double inertialKP, double inertialKI, double inertialKD,
                       long timeoutMili, int exitConditionExpectedPasses, Hardware robot);

void display_vision_error(int line, std::string msg);

/*
vision::signature BLUE GOAL (1, -2383, -1413, -1898, 8113, 11627, 9870, 3.100, 0);
vision::signature RED GOAL (2, 7761, 9095, 8428, -907, -503, -705, 4.400, 0);
vision::signature YELLOW GOAL (3, 1783, 2259, 2021, -3573, -3215, -3394, 5.500, 0);
vex::vision vision1 ( vex::PORT1, 29, BLUE GOAL, RED GOAL, YELLOW GOAL, SIG_4, SIG_5, SIG_6, SIG_7 );
*/
