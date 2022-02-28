#include "../include/main.h"
#include "GarglonConfiguration.hpp"

using namespace pros;

#define VISION_GUIDE 1
#define INERTIAL_GUIDE 2


// motor definitions //
pros::Motor left_front_motor(LEFT_FRONT_DRIVE_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_mid_motor(LEFT_MID_DRIVE_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_back_motor(LEFT_BACK_DRIVE_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front_motor(RIGHT_FRONT_DRIVE_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_mid_motor(RIGHT_MID_DRIVE_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_back_motor(RIGHT_BACK_DRIVE_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake_motor(INTAKE_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor arm_motor(ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Imu inertial_sensor(INERTIAL_SENSOR_PORT);
pros::ADIDigitalOut back_piston(PNEUMATIC_BACK_PORT);
pros::ADIDigitalOut front_piston(PNEUMATIC_FRONT_PORT);
pros::ADIDigitalOut top_piston(PNEUMATIC_TOP_PORT);
pros::ADIEncoder Y_encoder(Y_ENCODER_TOP_PORT, Y_ENCODER_BOTTOM_PORT, true);
pros::ADIPort touch_sensor(TOUCH_SENSOR_PORT, pros::E_ADI_ANALOG_IN);

pros::Controller master(CONTROLLER_MASTER);
pros::Vision front_vision (FRONT_VISION_PORT);
pros::Vision back_vision (BACK_VISION_PORT);

pros::vision_object_s_t closest_red_goal;
pros::vision_object_s_t closest_blue_goal;
pros::vision_object_s_t closest_yellow_goal;

// This variable defines the drifting angle during the period
// after Inertial sensor reset in Initialization() and before the starting of auton.
double sys_initial_to_auton_drifting = 0;
double sys_initial_robot_heading = 0;
double motion_initial_angle = ROBOT_INITIAL_HEADING;

turnDegreesPID_cmd_struct turnDegreesPID_Parameter =
                          {0, ON_SPOT_TURN, 0, CLOCKWISE, 6, 0, -20, 1000, 2};

void waitForTouch()
{
  while (true) {
    if (touch_sensor.get_value() <= 30) {
      break;
    }
    pros::delay(10);
  }
}

Hardware hardwareParameter{left_front_motor,
                           left_back_motor,
                           left_mid_motor,
                           right_front_motor,
                           right_back_motor,
                           right_mid_motor,
                           intake_motor,
                           arm_motor,
                           front_piston,
                           back_piston,
                           top_piston,
                           inertial_sensor,
                           Y_encoder
                          };

void set_drive_motor_speed(DriveSpeedConfig speedCon, Hardware robot)
{
    robot.leftFrontMotorLib.move(speedCon.leftFrontMotorSpeed);
    robot.leftBackMotorLib.move(speedCon.leftBackMotorSpeed);
    robot.leftMidMotorLib.move(speedCon.leftBackMotorSpeed);
    robot.rightFrontMotorLib.move(speedCon.rightFrontMotorSpeed);
    robot.rightBackMotorLib.move(speedCon.rightBackMotorSpeed);
    robot.rightMidMotorLib.move(speedCon.rightBackMotorSpeed);
}

void set_drive_motor_speed_zero(Hardware robot)
{
    robot.leftFrontMotorLib.move(0);
    robot.rightFrontMotorLib.move(0);
    robot.leftBackMotorLib.move(0);
    robot.rightBackMotorLib.move(0);
    robot.leftMidMotorLib.move(0);
    robot.rightMidMotorLib.move(0);
}

double get_robot_heading_lib(Hardware robot)
{
    return 0 - robot.inertialSensorLib.get_rotation()
           + sys_initial_robot_heading - sys_initial_to_auton_drifting;
}

double get_robot_heading_radians_lib(Hardware robot)
{
    float sys_initial_robot_heading = ROBOT_INITIAL_HEADING;
    return ((0 - robot.inertialSensorLib.get_rotation() + sys_initial_robot_heading
             -sys_initial_to_auton_drifting) / (180/3.1415926));
}


long average_long(long a1, long a2)
{
    return (a1 + a2) / 2;
}

double convert_target_to_relative_angle_lib(double currentRelativeHeading, double targetHeading)
{
    double targetRelativeHeading;
    double tmp = abs_double(getNormalizedAngle(targetHeading) - getNormalizedAngle(currentRelativeHeading));
    //always follow the smallest angle from currentHeading to targetRelativeHeading.
    if (tmp <= 180)
    {
        if (getNormalizedAngle(currentRelativeHeading) <= getNormalizedAngle(targetHeading))
        {
            targetRelativeHeading = currentRelativeHeading + tmp;
        }
        else
        {
            targetRelativeHeading = currentRelativeHeading - tmp;
        }
    }
    else
    {
        if (getNormalizedAngle(currentRelativeHeading) <= getNormalizedAngle(targetHeading))
        {
            targetRelativeHeading = currentRelativeHeading - (360 - tmp);
        }
        else
        {
            targetRelativeHeading = currentRelativeHeading + (360 - tmp);
        }
    }
    return targetRelativeHeading;
}

/***********************************************************************************************
* double convert_to_relative_angle_by_turn_direction_lib(double currentRelativeHeading, double targetHeading, int turnDirection){
*   Calculate the targetHeading relative to the currentHeading angle.
*   This function maintains the property: if turn clockwise, currentRelativeHeading > target_to_relative_angle.
*          If turn counter_clockwise, currentRelativeHeading < target_to_relative_angle.
*   double currentRelativeHeading: the current angle of the robot, this angle is based on the
*                                  defined coordinate system and is not normalized.
*   double targetHeading: this angle can be normalized or not normalized.
*   int turnDirection: either CLOCKWISE or COUNTER_CLOCKWISE
*   return: relative target angle to the currentHeading angle.
***********************************************************************************************/
double convert_to_relative_angle_by_turn_direction_lib(double currentRelativeHeading, double targetHeading, int turnDirection)
{
    double targetRelativeHeading;
    double normalTarget = getNormalizedAngle(targetHeading);           //0 <= angle < 360
    double normalCurrent = getNormalizedAngle(currentRelativeHeading); //0 <= angle < 360
    double diff = abs_double(normalTarget - normalCurrent);

    if (turnDirection == CLOCKWISE)
    {
        if (normalTarget <= normalCurrent)
            {
                targetRelativeHeading = currentRelativeHeading - diff;
            }
            else
            {
                targetRelativeHeading = currentRelativeHeading - (360 - diff);
            }
        }

        else if (turnDirection == COUNTER_CLOCKWISE)
        {
            if (normalTarget >= normalCurrent)
            {
                targetRelativeHeading = currentRelativeHeading + diff;
            }
            else
            {
                targetRelativeHeading = currentRelativeHeading + (360 - diff);
            }
        }

        else if (turnDirection == TURN_SMALLEST_ANGLE)
        {
            targetRelativeHeading = convert_target_to_relative_angle_lib(currentRelativeHeading, targetHeading);
        }

        else
        {
            targetRelativeHeading = 0;
        }

    return targetRelativeHeading;
}

/***************************************************************************************************************
* void goStraightCmPID()
*
* This function is based on the defined coordinate system.
*
* float cmDistance (any cm, preferably do not use decimals but there just in case)
* float robotHeading (can use a global variable as the parametre that changes after every turn; should be the intended heading of a turn)
* int robotSpeed (from 1-127 ALWAYS POSITIVE, also can be 1-254)
* int robotDirection (1 or MOVE_FORWARD = forward, -1 MOVE_BACKWARD = backward)
******************************************************************************************************************/
void goStraightCmPID_lib(double cmDistance, double robotHeadingLib, int maxPower, int robotDirection, double headingKP,
                         double headingKI, double headingKD, double distanceKP, double distanceKI, double distanceKD,
                         long timeoutMili, int exitConditionExpectedPasses, Hardware robot)
{

    long startingTime = pros::millis();
    long endTime = startingTime + timeoutMili; // timeout calculation
    long enterTime;
    long currentTime;
    long previousTime = 0;
    long substitutedTime;
    long delayTime = 10; // standardize time per cycle of while loop (so that kD is actually rate and not arbitrary)

    int max_speed = maxPower/3;
    int count = 1;

    double startingEncoder = robot.Y_encoderLib.get_value();
    double currentEncoderError = (cmDistance / (ENCODER_CIRCUMFERENCE_CM) * 360 * robotDirection);
    double previousEncoderError = currentEncoderError;
    double targetEncoderDegrees = currentEncoderError + startingEncoder;
  //  lcd::print(1, "target: %f", targetEncoderDegrees);
  //  waitForTouch();
    double currentEncoder = robot.Y_encoderLib.get_value();

    double currentHeading = get_robot_heading_lib(robot);
    motion_initial_angle = currentHeading;
    robotHeadingLib = convert_target_to_relative_angle_lib(currentHeading, robotHeadingLib);
    double priorHeading = currentHeading;
    double currentHeadingError = 0;
    double headingChange = 0;
    double accumulatedHeadingError = 0;
    double accumulatedEncoderError = 0;

    double distanceProportionalCorrection = 0;
    double distanceIntegralCorrection = 0;
    double distanceDerivativeCorrection = 0;
    double totalDistanceCorrection = 0;

    double headingProportionalCorrection = 0;
    double headingIntegralCorrection = 0;
    double headingDerivativeCorrection = 0;
    double totalHeadingCorrection = 0;

    long exitConditionPasses = 0; // number of times the PID caused the robot to go over the target distance

    DriveSpeedConfig distanceSpeedConfig;
    DriveSpeedConfig headingSpeedConfig;
    DriveSpeedConfig totalSpeedConfig;


    while (true)
    {
      count++;
      max_speed = maxPower/3 + maxPower * 2 / 3 * count / 10;
      max_speed = (int) truncateNumber(max_speed, maxPower);

        enterTime = pros::millis();
        currentHeading = get_robot_heading_lib(robot);
        currentEncoder = robot.Y_encoderLib.get_value();
        currentEncoderError = targetEncoderDegrees - currentEncoder;
        if (currentEncoderError == 0)
        {
            exitConditionPasses++;
        }
        else if (previousEncoderError > 0 && currentEncoderError < 0)
        { // uses error to tell how close the robot is to the target
            exitConditionPasses++;
        }
        else if (currentEncoderError > 0 && previousEncoderError < 0)
        {
            exitConditionPasses++;
        }

        if (exitConditionPasses >= exitConditionExpectedPasses)
        { // this will determine the number of times you want the bot to cross the target line
          //        pros::lcd::print(3, "exit by pass number"); // distance reached multiple times
            set_drive_motor_speed_zero(robot);
            lcd::print(5, "EXIT CONDITION");
            break;
        }
        else if (pros::millis() >= endTime)
        {
            //        pros::lcd::print(3, "exit by timeout"); // timer up
            set_drive_motor_speed_zero(robot);
            lcd::print(5, "EXIT TIMER");
            break;
        }
        //HEADING CORRECTION//
        currentHeadingError = robotHeadingLib - currentHeading; //finds the diff between current and target heading
        headingProportionalCorrection = currentHeadingError * headingKP;

        accumulatedHeadingError = currentHeadingError + accumulatedHeadingError;
        headingIntegralCorrection = accumulatedHeadingError * headingKI;
        //Derivative
        headingChange = currentHeading - priorHeading;
        headingDerivativeCorrection = headingChange * headingKD;
        //all
        totalHeadingCorrection = headingProportionalCorrection + headingIntegralCorrection + headingDerivativeCorrection;
        totalHeadingCorrection = truncateNumber(totalHeadingCorrection, max_speed);
        priorHeading = currentHeading; //resets the heading for next loop

        //DISTANCE CORRECTION//
        //kP//
        distanceProportionalCorrection = currentEncoderError * distanceKP; // simple proportional calculation
        //kI//
        accumulatedEncoderError = accumulatedEncoderError + currentEncoderError;
        distanceIntegralCorrection = accumulatedEncoderError * distanceKI;
        //kD//
        distanceDerivativeCorrection = (currentEncoderError - previousEncoderError) * distanceKD;                 // error difference in 2 measurements * the kD.
        totalDistanceCorrection = distanceProportionalCorrection + distanceIntegralCorrection + distanceDerivativeCorrection; // sum of the 2 corrections
        totalDistanceCorrection = truncateNumber(totalDistanceCorrection, max_speed);
        pros::lcd::print(2, "s=%.2f, c=%.2f, tar=%.2f", totalDistanceCorrection, currentEncoder, targetEncoderDegrees);

//lcd::print(1, "target: %f", totalDistanceCorrection);

        distanceSpeedConfig = assignDriveSpeed(totalDistanceCorrection, totalDistanceCorrection,
                                               totalDistanceCorrection, totalDistanceCorrection);

        headingSpeedConfig = assignDriveSpeed(0 - totalHeadingCorrection, totalHeadingCorrection,
                                              0 - totalHeadingCorrection, totalHeadingCorrection);

        totalSpeedConfig = proportional_projection_two_speed_config(distanceSpeedConfig,
                           headingSpeedConfig, max_speed);
        set_drive_motor_speed(totalSpeedConfig, robot);

        pros::lcd::print(7, "torque: %f", (left_mid_motor.get_torque() + right_mid_motor.get_torque())/2);

        previousEncoderError = currentEncoderError;
        previousTime = currentTime;

        substitutedTime = pros::millis() - enterTime;
        pros::delay(delayTime - substitutedTime); // the targeted delay time needs to be consistent every loop by finding how long the execution within the loop took and then subtracting that from the preferred delay time
    }
    motion_initial_angle = get_robot_heading_lib(robot);
}

/***********************************************************************************************************************************
* This function is based on the defined coordinate system.
*
* turnType has 2 inputs, ONE_WHEEL_TURN_FORWARD, ONE_WHEEL_TURN_BACKWARD, and ON_SPOT_TURN which will call for different PID values,
* the two wheel turn will be a separate function and means 2 alternate values for powers
* PLEASE USE ANYWHERE FROM 0 TO 360, the angle out of this range will be normalized into this range
* turnDirection is CLOCKWISE or COUNTER_CLOCKWISE
*************************************************************************************************************************************/
void turnDegreesPID_lib(double targetHeadingLib, double turnType, long maxPower, int turnDirection, double headingKP, double headingKI,
                        double headingKD, long timeoutMili, int exitConditionExpectedPasses, Hardware config)
{
    double startingHeading = get_robot_heading_lib(config);
    double targetHeading = convert_to_relative_angle_by_turn_direction_lib(startingHeading, targetHeadingLib, turnDirection);
    lcd::print(1, "targetheading: %f", targetHeading);

    long startingTime = pros::millis();
    long endTime = startingTime + timeoutMili;

    double headingProportionalCorrection = 0;
    double headingIntegralCorrection = 0;
    double headingDerivativeCorrection = 0;
    double currentHeading = startingHeading;
    double priorHeading = startingHeading;
    double currentHeadingError = 0;
    double priorHeadingError = 0;
    double headingChange = 0;
    double accumulatedHeadingError = 0;

    long substitutedTime;
    long enterTime;
    long delayTime = 10;

    double totalHeadingCorrection = 0;
    double totalDistanceCorrection = 0;

    double leftMotorFinalPower = 0;
    double rightMotorFinalPower = 0;

    long exitConditionPasses = 0; // number of times the PID caused the robot to go over the target distance

    while (true)
    {
        currentHeading = get_robot_heading_lib(config);
        lcd::print(2, "currentHeading: %f", currentHeading);
        currentHeadingError = targetHeading - currentHeading;
        enterTime = pros::millis();

        if (currentHeadingError == 0)
        {
            exitConditionPasses++;
        }
        else if (priorHeadingError > 0 && currentHeadingError < 0)
        { // uses error to tell how close the robot is to the target
            exitConditionPasses++;
        }
        else if (currentHeadingError > 0 && priorHeadingError < 0)
        {
            exitConditionPasses++;
        }
        //    pros::lcd::print(3, "heading = %f", currentHeading); // distance reached multiple times
        //    pros::lcd::print(4, "heading error = %f", currentHeadingError); // distance reached multiple times
        //    pros::lcd::print(5, "exit passed %d times", exitConditionPasses); // distance reached multiple times
        if (exitConditionPasses >= exitConditionExpectedPasses)
        { // this will determine the number of times you want the bot to cross the target line
          //      pros::lcd::print(6, "heading checkpoint");  // distance reached multiple times
            left_front_motor.move(0);
            left_back_motor.move(0);
            right_front_motor.move(0);
            right_back_motor.move(0);
            pros::lcd::print(5, "TURN EXIT CONDITION");
            break;
        }

        else if (pros::millis() >= endTime)
        {
          //      pros::lcd::print(6, "timeout checkpoint"); // timer up
            left_front_motor.move(0);
            left_back_motor.move(0);
            right_front_motor.move(0);
            right_back_motor.move(0);
            pros::lcd::print(5, "TURN EXIT TIMER");
            break;
        }
        //Proportional
        headingProportionalCorrection = currentHeadingError * headingKP;
        //Integral
        accumulatedHeadingError = currentHeadingError + accumulatedHeadingError;
        headingIntegralCorrection = accumulatedHeadingError * headingKI;
        //Derivative
        headingChange = currentHeading - priorHeading;
        headingDerivativeCorrection = headingChange * headingKD;
        //all
        totalHeadingCorrection = headingProportionalCorrection + headingIntegralCorrection + headingDerivativeCorrection;
        priorHeading = currentHeading; //resets the heading for next loop
        priorHeadingError = currentHeadingError;

        if (totalHeadingCorrection >= maxPower)
        { // ensures the robot speed stays at the max power you set, it can exceed it if there needs to be heading correction
          totalHeadingCorrection = maxPower;
        }

        if (turnType == ONE_WHEEL_TURN_FORWARD)
        {
            if (turnDirection == COUNTER_CLOCKWISE)
            {
                leftMotorFinalPower = 0;
                rightMotorFinalPower = totalHeadingCorrection;
            }
            else if (turnDirection == CLOCKWISE)
            {
                leftMotorFinalPower = 0 - totalHeadingCorrection;
                rightMotorFinalPower = 0;
            }
        }
        else if (turnType == ONE_WHEEL_TURN_BACKWARD)
        {
            if (turnDirection == COUNTER_CLOCKWISE)
            {
                leftMotorFinalPower = 0 - totalHeadingCorrection;
                rightMotorFinalPower = 0;
            }
            else if (turnDirection == CLOCKWISE)
            {
                leftMotorFinalPower = 0;
                rightMotorFinalPower = totalHeadingCorrection;
            }
        }
        else if (turnType == ON_SPOT_TURN)
        {
          leftMotorFinalPower = 0 - totalHeadingCorrection;
          rightMotorFinalPower = totalHeadingCorrection;
        }

        config.leftFrontMotorLib.move(leftMotorFinalPower);
        config.leftBackMotorLib.move(leftMotorFinalPower);
        config.leftMidMotorLib.move(leftMotorFinalPower);
        config.rightFrontMotorLib.move(rightMotorFinalPower);
        config.rightBackMotorLib.move(rightMotorFinalPower);
        config.rightMidMotorLib.move(rightMotorFinalPower);

        substitutedTime = pros::millis() - enterTime;
        pros::delay(delayTime - substitutedTime);
    }
  //  pros::lcd::print(7, "function time = %d", (pros::millis() - startingTime));
}

/******************************************

******************************************/
void twoWheelTurnDegreesPID(double targetHeadingLib, double turnType, double smallerPower,
                            double largerPower, int turnDirection, double headingKP,
                            double headingKI, double headingKD, long timeoutMili,
                            int exitConditionExpectedPasses, Hardware config)
{

    double startingHeading = get_robot_heading_lib(config);
    double targetHeading = convert_target_to_relative_angle_lib(startingHeading, targetHeadingLib);

    long startingTime = pros::millis();
    long endTime = startingTime + timeoutMili;

    double headingProportionalCorrection = 0;
    double headingIntegralCorrection = 0;
    double headingDerivativeCorrection = 0;

    double currentHeading = config.inertialSensorLib.get_rotation();
    double currentHeadingError = 0;
    double priorHeading = config.inertialSensorLib.get_rotation();
    double priorHeadingError = 0;
    double headingChange = 0;
    double accumulatedHeadingError = 0;

    long substitutedTime;
    long enterTime;
    long delayTime = 10;

    double totalHeadingCorrection = 0;
    double totalDistanceCorrection = 0;

    double leftMotorFinalPower = 0;
    double rightMotorFinalPower = 0;

    long exitConditionPasses = 0; // number of times the PID caused the robot to go over the target distance

    while (true)
    {
        currentHeading = get_robot_heading_lib(config);

        currentHeadingError = targetHeading - currentHeading;
        enterTime = pros::millis();

        if (currentHeadingError == 0)
        {
            exitConditionPasses++;
        }
        else if (priorHeadingError > 0 && currentHeadingError < 0)
        {   // uses error to tell how close the robot is to the target
            exitConditionPasses++;
        }
        else if (currentHeadingError > 0 && priorHeadingError < 0)
        {
            exitConditionPasses++;
        }

        if (exitConditionPasses >= exitConditionExpectedPasses)
        {   // this will determine the number of times you want the bot to cross the target line
            //      pros::lcd::print(7, "heading checkpoint"); // distance reached multiple times
            left_front_motor.move(0);
            left_back_motor.move(0);
            right_front_motor.move(0);
            right_back_motor.move(0);
            break;
        }
        else if (pros::millis() >= endTime)
        {
            //      pros::lcd::print(7, "timeout checkpoint"); // timer up
            left_front_motor.move(0);
            left_back_motor.move(0);
            right_front_motor.move(0);
            right_back_motor.move(0);
            break;
        }

        headingProportionalCorrection = currentHeadingError * headingKP;
        //      pros::lcd::print(3, "kP correction = %f", proportionalCorrection);

        //Integral
        accumulatedHeadingError = currentHeadingError + accumulatedHeadingError;
        headingIntegralCorrection = accumulatedHeadingError * headingKI;
        //      pros::lcd::print(4, "kI correction = %f", integralCorrection);

        //Derivative
        headingChange = currentHeading - priorHeading;
        headingDerivativeCorrection = headingChange * headingKD;
        //      pros::lcd::print(5, "kD correction = %f", derivativeCorrection);
        //      pros::lcd::print(6, "accumulation = %f", accumulatedHeadingError);

        //all
        totalHeadingCorrection = headingProportionalCorrection + headingIntegralCorrection
                                 + headingDerivativeCorrection;

        priorHeading = currentHeading; //resets the heading for next loop
        priorHeadingError = currentHeadingError;

        if (totalHeadingCorrection >= largerPower)
        { // ensures the robot speed stays at the max power you set, it can exceed it if there needs to be heading correction
            totalHeadingCorrection = largerPower;
        }
        else if (totalHeadingCorrection <= (-1 * abs_double(largerPower)))
        {
            totalHeadingCorrection = (-1 * abs_double(largerPower));
        }

        if (turnType == TWO_WHEEL_TURN_FORWARD)
        {
            if (turnDirection == CLOCKWISE)
            {
              leftMotorFinalPower = -1 * totalHeadingCorrection;
              rightMotorFinalPower = -1 * totalHeadingCorrection * (smallerPower / largerPower);
            }
            else if (turnDirection == COUNTER_CLOCKWISE)
            {
              leftMotorFinalPower = totalHeadingCorrection * (smallerPower / largerPower);
                rightMotorFinalPower = totalHeadingCorrection;
            }
        }
        else if (turnType == TWO_WHEEL_TURN_BACKWARD)
        {
            if (turnDirection == CLOCKWISE)
            {
              leftMotorFinalPower = totalHeadingCorrection * (smallerPower / largerPower);
                rightMotorFinalPower = totalHeadingCorrection;
            }
            else if (turnDirection == COUNTER_CLOCKWISE)
            {
              leftMotorFinalPower = -1 * totalHeadingCorrection;
                rightMotorFinalPower = -1 * totalHeadingCorrection * (smallerPower / largerPower);
            }
        }
        //    pros::lcd::print(5, "right = %f", rightMotorFinalPower); // distance reached multiple times
        //    pros::lcd::print(6, "left = %f", leftMotorFinalPower); // distance reached multiple times

        config.leftFrontMotorLib.move(leftMotorFinalPower);
        config.leftBackMotorLib.move(leftMotorFinalPower);
        config.leftMidMotorLib.move(leftMotorFinalPower);
        config.rightFrontMotorLib.move(rightMotorFinalPower);
        config.rightBackMotorLib.move(rightMotorFinalPower);
        config.rightMidMotorLib.move(rightMotorFinalPower);

        substitutedTime = pros::millis() - enterTime;
        pros::delay(delayTime - substitutedTime);

        pros::lcd::print(3, "%f", leftMotorFinalPower);
    }
}


//vision sensor is 6cm to middle of robot
double get_mobile_goal_distance_vision(double object_width){
  double width = object_width;
  return 34444.0 * pow(width, -1.343);
}

//vision sensor is 6cm to middle of robot
int get_middle_coordinate(int object_width){
  double width = object_width;
  return (int)(83.285 * pow(width, 0.1384) + 0.5);

  //y = 83.285 x^0.1384
}

int get_left_coordinate(int object_width){
  double width = object_width;
  return (int)(0.0005 * width * width - 0.5602 * width + 162.92 + 0.5);
//  return (int)(0.0005 * width * width - 0.5602 * width + 151.92 + 0.5);
}


/***************************************************************************************************
* ball_color_signature must be assigned as one of RED_BALL_SIG or BLUE_BALL_SIG
****************************************************************************************************/
void goStraightCm_Front_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                         int goal_color_signature, Vision vision_sensor,
                         double headingKP, double headingKI, double headingKD,
                         double distanceKP, double distanceKI, double distanceKD,
                         double visionKP, double visionKI, double visionKD,
                         long timeoutMili, int exitConditionExpectedPasses, Hardware robot){

  if(goal_color_signature != DETECT_RED_GOAL_SIG && goal_color_signature != DETECT_BLUE_GOAL_SIG
     && goal_color_signature != DETECT_YELLOW_GOAL_SIG) {
    return;
  }

  pros::vision_object_s_t closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
  int guideType = VISION_GUIDE;
  int preGuideType = VISION_GUIDE;

  if(errno == EAGAIN || errno == EINVAL || errno == EDOM || errno == EAGAIN){
    guideType = INERTIAL_GUIDE;
    preGuideType = INERTIAL_GUIDE;
    pros::lcd::print(1, "inertial guide ");
  } else{
    //vision guide, does nothing
    pros::lcd::print(1, "vision guide ");
  }

  long startingTime = pros::millis();
  long endTime = startingTime + timeoutMili;
  long enterTime;
  long currentTime;
  long previousTime = 0;
  long substitutedTime;
  long delayTime = 10;

  cmDistance = std::abs(cmDistance);
  double currentEncoder = Y_encoder.get_value();
  double targetEncoderDegrees = cmDistance/(6.985*3.14) * 360 + currentEncoder;
  double currentEncoderError = 0;
  double previousEncoderError = 0;
  double accumulatedEncoderError = 0;
  double distanceProportionalCorrection = 0;
  double distanceIntegralCorrection = 0;
  double distanceDerivativeCorrection = 0;
  double totalDistanceCorrection = 0;

  double targetVisionHeading = 0;
  double currentVisionHeading = closest_goal.x_middle_coord;
  double currentVisionHeadingError = 0;
  double priorVisionHeading = currentVisionHeading;
  double headingVisionChange = 0;
  double accumulatedVisionHeadingError = 0;

  double currentInertialHeading = get_robot_heading_lib(robot);
  double targetInertialHeading = convert_target_to_relative_angle_lib(currentInertialHeading, robotInertialHeadingLib);
  double currentInertialHeadingError = 0;
  double priorInertialHeading = currentVisionHeading;
  double headingInertialChange = 0;
  double accumulatedInertialHeadingError = 0;

  double headingProportionalCorrection = 0;
  double headingIntegralCorrection = 0;
  double headingDerivativeCorrection = 0;
  double totalHeadingCorrection = 0;

  long exitConditionPasses = 0; // number of times the PID caused the robot to go over the target distance
  DriveSpeedConfig distanceSpeedConfig;
  DriveSpeedConfig headingSpeedConfig;
  DriveSpeedConfig totalSpeedConfig;

  long n = 0;
  long num_gyro = 0;
  long num_vision = 0;
  int target_middle = 0;

  while(true){
      enterTime = pros::millis();
      closest_goal.width = -10;
      closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
      target_middle = get_middle_coordinate(closest_goal.width);
      targetVisionHeading = target_middle;
//      pros::lcd::print(1, "w = %d%", closest_goal.width);
      if(errno == EAGAIN){      //|| errno == ENODEV){  // read vision sensor error
        n++;
        preGuideType = guideType;
        guideType = INERTIAL_GUIDE;
        pros::lcd::print(2, "ERR: no vision %d %d %d", errno, ENODEV, n);
      } else { //read vision sensor successfully
        if (closest_goal.width >= 20 && closest_goal.x_middle_coord >= target_middle - 2
                                           && closest_goal.x_middle_coord <= target_middle + 2){
          targetInertialHeading = get_robot_heading_lib(robot);
          n++;
        }

        if (closest_goal.x_middle_coord <= 10 || closest_goal.x_middle_coord >= 306
                                                    || closest_goal.width <= 20){
          if(guideType == VISION_GUIDE){
            guideType = INERTIAL_GUIDE;   //inertial sensor guide
          }
        } else {
          if(guideType == INERTIAL_GUIDE){
            guideType = VISION_GUIDE;   //vision sensor guide
          }
        }
      }

      currentEncoder = Y_encoder.get_value();
      currentEncoderError = targetEncoderDegrees - currentEncoder;
      if(currentEncoderError == 0){
        exitConditionPasses ++;
      }
      else if(previousEncoderError > 0 && currentEncoderError < 0){  // uses error to tell how close the robot is to the target
        exitConditionPasses ++;
      }
      else if(currentEncoderError > 0 && previousEncoderError < 0){
        exitConditionPasses ++;
      }

      if(exitConditionPasses >= exitConditionExpectedPasses){ // this will determine the number of times you want the bot to cross the target line
        set_drive_motor_speed_zero(robot);
        break;
      } else if(pros::millis() >= endTime){
        set_drive_motor_speed_zero(robot);
        break;
      }

      //HEADING CORRECTION/////////////////////////////////////////////////////////////////////////////////////////
      if(guideType == INERTIAL_GUIDE){
        num_gyro++;
        currentInertialHeading = get_robot_heading_lib(robot);
        if(preGuideType == VISION_GUIDE){
          priorInertialHeading = currentInertialHeading;
        }
        currentInertialHeadingError = targetInertialHeading - currentInertialHeading; //finds the diff between current and target heading
        headingProportionalCorrection = currentInertialHeadingError * headingKP;

        accumulatedInertialHeadingError = currentInertialHeadingError + accumulatedInertialHeadingError;
        headingIntegralCorrection = accumulatedInertialHeadingError * headingKI;

        headingInertialChange = currentInertialHeading - priorInertialHeading;
        headingDerivativeCorrection = headingInertialChange * headingKD;
        //all
        totalHeadingCorrection = (headingProportionalCorrection + headingIntegralCorrection +
                                      headingDerivativeCorrection);
        priorInertialHeading = currentInertialHeading; //resets the heading for next loop
      } else if (guideType == VISION_GUIDE){
        num_vision++;
        currentVisionHeading = closest_goal.x_middle_coord;
        if(preGuideType == INERTIAL_GUIDE){
          priorVisionHeading = currentVisionHeading;
        }

        currentVisionHeadingError =  targetVisionHeading - currentVisionHeading;//finds the diff between current and target heading
        headingProportionalCorrection = currentVisionHeadingError * visionKP;
        //Integral
        accumulatedVisionHeadingError = currentVisionHeadingError + accumulatedVisionHeadingError;
        headingIntegralCorrection = accumulatedVisionHeadingError * visionKI;
        //Derivative
        headingVisionChange = currentVisionHeading - priorVisionHeading;
        headingDerivativeCorrection = headingVisionChange * visionKD;
        //all
        totalHeadingCorrection = headingProportionalCorrection + headingIntegralCorrection + headingDerivativeCorrection;
        priorVisionHeading = currentVisionHeading; //resets the heading for next loop
      }

      totalHeadingCorrection = truncateNumber(totalHeadingCorrection, maxSpeed/2);

      //DISTANCE CORRECTION////////////////////////////////////////////////////////////////////////////////////////////////////
      distanceProportionalCorrection = currentEncoderError * distanceKP; // simple proportional calculation
      accumulatedEncoderError = accumulatedEncoderError + currentEncoderError;
      distanceIntegralCorrection = accumulatedEncoderError * distanceKI * delayTime;
      distanceDerivativeCorrection = (currentEncoderError - previousEncoderError) * distanceKD / delayTime; // error difference in 2 measurements * the kD.
      totalDistanceCorrection = distanceProportionalCorrection + distanceIntegralCorrection + distanceDerivativeCorrection; // sum of the 2 corrections
      totalDistanceCorrection = truncateNumber(totalDistanceCorrection, maxSpeed);

      headingSpeedConfig  = assignDriveSpeed(0 - totalHeadingCorrection, totalHeadingCorrection,
                                             0 - totalHeadingCorrection, totalHeadingCorrection);
      distanceSpeedConfig  = assignDriveSpeed(totalDistanceCorrection, totalDistanceCorrection,
                                              totalDistanceCorrection, totalDistanceCorrection);
      totalSpeedConfig = proportional_projection_two_speed_config(distanceSpeedConfig, headingSpeedConfig, maxSpeed);
      set_drive_motor_speed(totalSpeedConfig, robot);

      preGuideType = guideType;
      previousEncoderError = currentEncoderError;
      previousTime = currentTime;
      substitutedTime = pros::millis() - enterTime;
      pros::delay(delayTime - substitutedTime); // the targeted delay time needs to be consistent every loop by finding how long the execution within the loop took and then subtracting that from the preferred delay time
    }
}

void goStraightCm_Back_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                         int goal_color_signature, Vision vision_sensor,
                         double headingKP, double headingKI, double headingKD,
                         double distanceKP, double distanceKI, double distanceKD,
                         double visionKP, double visionKI, double visionKD,
                         long timeoutMili, int exitConditionExpectedPasses, Hardware robot){

  if(goal_color_signature != DETECT_RED_GOAL_SIG && goal_color_signature != DETECT_BLUE_GOAL_SIG
     && goal_color_signature != DETECT_YELLOW_GOAL_SIG) {
    return;
  }

  pros::vision_object_s_t closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
  int guideType = VISION_GUIDE;
  int preGuideType = VISION_GUIDE;

  if(errno == EAGAIN || errno == EINVAL || errno == EDOM || errno == EAGAIN){
    guideType = INERTIAL_GUIDE;
    preGuideType = INERTIAL_GUIDE;
    pros::lcd::print(1, "inertial guide ");
  } else{
    //vision guide, does nothing
    pros::lcd::print(1, "vision guide ");
  }

  long startingTime = pros::millis();
  long endTime = startingTime + timeoutMili;
  long enterTime;
  long currentTime;
  long previousTime = 0;
  long substitutedTime;
  long delayTime = 10;

  cmDistance = std::abs(cmDistance);
  double currentEncoder = Y_encoder.get_value();
  double targetEncoderDegrees = - cmDistance/(6.985*3.14) * 360 + currentEncoder;
  double currentEncoderError = 0;
  double previousEncoderError = 0;
  double accumulatedEncoderError = 0;
  double distanceProportionalCorrection = 0;
  double distanceIntegralCorrection = 0;
  double distanceDerivativeCorrection = 0;
  double totalDistanceCorrection = 0;

  double targetVisionHeading = 158;
  double currentVisionHeading = closest_goal.x_middle_coord;
  double currentVisionHeadingError = 0;
  double priorVisionHeading = currentVisionHeading;
  double headingVisionChange = 0;
  double accumulatedVisionHeadingError = 0;

  double currentInertialHeading = get_robot_heading_lib(robot);
  double targetInertialHeading = convert_target_to_relative_angle_lib(currentInertialHeading, robotInertialHeadingLib);
  double currentInertialHeadingError = 0;
  double priorInertialHeading = currentVisionHeading;
  double headingInertialChange = 0;
  double accumulatedInertialHeadingError = 0;

  double headingProportionalCorrection = 0;
  double headingIntegralCorrection = 0;
  double headingDerivativeCorrection = 0;
  double totalHeadingCorrection = 0;

  long exitConditionPasses = 0; // number of times the PID caused the robot to go over the target distance
  DriveSpeedConfig distanceSpeedConfig;
  DriveSpeedConfig headingSpeedConfig;
  DriveSpeedConfig totalSpeedConfig;

  long n = 0;
  long num_gyro = 0;
  long num_vision = 0;

  while(true){
      enterTime = pros::millis();
      closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
      if(errno == EAGAIN){ // || errno == ENODEV){  // read vision sensor error
        preGuideType = guideType;
        guideType = INERTIAL_GUIDE;
        pros::lcd::print(2, "ERR: no vision sensor ");
      } else { //read vision sensor successfully
        if (closest_goal.width >= 20 && closest_goal.x_middle_coord >= 157
                                           && closest_goal.x_middle_coord <= 159){
          targetInertialHeading = get_robot_heading_lib(robot);
          n++;
        }

        if (closest_goal.x_middle_coord <= 10 || closest_goal.x_middle_coord >= 306
                                                    || closest_goal.width <= 20){
          if(guideType == VISION_GUIDE){
            guideType = INERTIAL_GUIDE;   //inertial sensor guide
          }
        } else {
          if(guideType == INERTIAL_GUIDE){
            guideType = VISION_GUIDE;   //vision sensor guide
          }
        }
      }

      currentEncoder = Y_encoder.get_value();
      currentEncoderError = targetEncoderDegrees - currentEncoder;
      if(currentEncoderError == 0){
        exitConditionPasses ++;
      }
      else if(previousEncoderError > 0 && currentEncoderError < 0){  // uses error to tell how close the robot is to the target
        exitConditionPasses ++;
      }
      else if(currentEncoderError > 0 && previousEncoderError < 0){
        exitConditionPasses ++;
      }

      if(exitConditionPasses >= exitConditionExpectedPasses){ // this will determine the number of times you want the bot to cross the target line
        set_drive_motor_speed_zero(robot);
        pros::lcd::print(5, "EXIT CONDITION");
        break;
      } else if(pros::millis() >= endTime){
        set_drive_motor_speed_zero(robot);
        pros::lcd::print(5, "EXIT TIMER");
        break;
      }

      //HEADING CORRECTION/////////////////////////////////////////////////////////////////////////////////////////
      if(guideType == INERTIAL_GUIDE){
        num_gyro++;
        currentInertialHeading = get_robot_heading_lib(robot);
        if(preGuideType == VISION_GUIDE){
          priorInertialHeading = currentInertialHeading;
        }
        currentInertialHeadingError = targetInertialHeading - currentInertialHeading; //finds the diff between current and target heading
        headingProportionalCorrection = currentInertialHeadingError * headingKP;

        accumulatedInertialHeadingError = currentInertialHeadingError + accumulatedInertialHeadingError;
        headingIntegralCorrection = accumulatedInertialHeadingError * headingKI;

        headingInertialChange = currentInertialHeading - priorInertialHeading;
        headingDerivativeCorrection = headingInertialChange * headingKD;
        //all
        totalHeadingCorrection = (headingProportionalCorrection + headingIntegralCorrection +
                                      headingDerivativeCorrection);
        priorInertialHeading = currentInertialHeading; //resets the heading for next loop
      } else if (guideType == VISION_GUIDE){
        num_vision++;
        currentVisionHeading = closest_goal.x_middle_coord;
        if(preGuideType == INERTIAL_GUIDE){
          priorVisionHeading = currentVisionHeading;
        }

        currentVisionHeadingError =  targetVisionHeading - currentVisionHeading;//finds the diff between current and target heading
        headingProportionalCorrection = currentVisionHeadingError * visionKP;
        //Integral
        accumulatedVisionHeadingError = currentVisionHeadingError + accumulatedVisionHeadingError;
        headingIntegralCorrection = accumulatedVisionHeadingError * visionKI;
        //Derivative
        headingVisionChange = currentVisionHeading - priorVisionHeading;
        headingDerivativeCorrection = headingVisionChange * visionKD;
        //all
        totalHeadingCorrection = 0 - (headingProportionalCorrection + headingIntegralCorrection + headingDerivativeCorrection); // PUT 0- BECAUSE VISION SENSOR IS INSTALLED UPSIDE DOWN
        priorVisionHeading = currentVisionHeading; //resets the heading for next loop
      }

      totalHeadingCorrection = truncateNumber(totalHeadingCorrection, maxSpeed/2);

      //DISTANCE CORRECTION////////////////////////////////////////////////////////////////////////////////////////////////////
      distanceProportionalCorrection = currentEncoderError * distanceKP; // simple proportional calculation
      accumulatedEncoderError = accumulatedEncoderError + currentEncoderError;
      distanceIntegralCorrection = accumulatedEncoderError * distanceKI * delayTime;
      distanceDerivativeCorrection = (currentEncoderError - previousEncoderError) * distanceKD / delayTime; // error difference in 2 measurements * the kD.
      totalDistanceCorrection = distanceProportionalCorrection + distanceIntegralCorrection + distanceDerivativeCorrection; // sum of the 2 corrections
      totalDistanceCorrection = truncateNumber(totalDistanceCorrection, maxSpeed);

      headingSpeedConfig  = assignDriveSpeed(0 - totalHeadingCorrection, totalHeadingCorrection,
                                             0 - totalHeadingCorrection, totalHeadingCorrection);
      distanceSpeedConfig  = assignDriveSpeed(totalDistanceCorrection, totalDistanceCorrection,
                                              totalDistanceCorrection, totalDistanceCorrection);
      totalSpeedConfig = proportional_projection_two_speed_config(distanceSpeedConfig, headingSpeedConfig, maxSpeed);
      set_drive_motor_speed(totalSpeedConfig, robot);

      preGuideType = guideType;
      previousEncoderError = currentEncoderError;
      previousTime = currentTime;
      substitutedTime = pros::millis() - enterTime;
      pros::delay(delayTime - substitutedTime); // the targeted delay time needs to be consistent every loop by finding how long the execution within the loop took and then subtracting that from the preferred delay time
    }
}


// void assign_turnDegreesPID_Parameter(double targetHeadingLib, double turnType, long maxPower, int turnDirection,
//                                      double headingKP, double headingKI, double headingKD, long timeoutMili,
//                                      int exitConditionExpectedPasses) {
//   turnDegreesPID_Parameter.targetHeadingLib = targetHeadingLib;
//   turnDegreesPID_Parameter.turnType = turnType;
//   turnDegreesPID_Parameter.maxPower = maxPower;
//   turnDegreesPID_Parameter.turnDirection = turnDirection;
//   turnDegreesPID_Parameter.headingKP = headingKP;
//   turnDegreesPID_Parameter.headingKI = headingKI;
//   turnDegreesPID_Parameter.headingKD = headingKD;
//   turnDegreesPID_Parameter.timeoutMili = timeoutMili;
//   turnDegreesPID_Parameter.exitConditionExpectedPasses = exitConditionExpectedPasses;
// }


/***************************************************************************************************
* ball_color_signature must be assigned as one of RED_BALL_SIG or BLUE_BALL_SIG
****************************************************************************************************/
void search_Goal_Back_Vision(int maxSpeed, int searchDirection, double maxSearchintAngle,
                       double goal_min_width, int goal_color_signature, pros::Vision vision_sensor,
                       double visionKP, double visionKI, double visionKD,
                       double inertialKP, double inertialKI, double inertialKD,
                       long timeoutMili, int exitConditionExpectedPasses, Hardware robot){

  if(goal_color_signature != DETECT_RED_GOAL_SIG && goal_color_signature != DETECT_BLUE_GOAL_SIG
                      && goal_color_signature != DETECT_YELLOW_GOAL_SIG) {
    pros::lcd::print(1, "Invalid search signature");
    return;
  }
  pros::vision_object_s_t closest_goal;

  int maxPower = truncateNumber(abs(maxSpeed), 127);
  long endTimeMillis = pros::millis() + timeoutMili;


  turnDegreesPID_Parameter = {180, ON_SPOT_TURN, 0, CLOCKWISE, 6, 0, -20, 1000, 2};
  pros::Task background(background_execution_turnDegreesPID_lib, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "turn");


  double currentInertialHeading = get_robot_heading_lib(robot);
  double priorInertialHeading = currentInertialHeading;
  double currentInertialHeadingError;
  double priorInertialHeadingError;
  double accumulatedInertialHeadingError;
  double headingInertialChange;
  double maxTargetInertialHeading = currentInertialHeading;
  double inertialGuideTarget = currentInertialHeading;
  DriveSpeedConfig spinConfig;

  if(searchDirection == CLOCKWISE){
    maxTargetInertialHeading = currentInertialHeading - fabs(maxSearchintAngle);
    spinConfig = assignDriveSpeed(maxPower, -maxPower, maxPower, -maxPower);
    pros::lcd::print(1, "%f", maxTargetInertialHeading);
  } else if(searchDirection == COUNTER_CLOCKWISE){
    maxTargetInertialHeading = currentInertialHeading + fabs(maxSearchintAngle);
    spinConfig = assignDriveSpeed(-maxPower, maxPower, -maxPower, maxPower);
    pros::lcd::print(1, "%f", maxTargetInertialHeading);
  } else {
    pros::lcd::print(1, "Invalid search direction");
    return;
  }

  double targetVisionHeading = 158;
  double currentVisionHeading;
  double priorVisionHeading = currentVisionHeading;
  double currentVisionHeadingError = 0;
  double priorVisionHeadingError = 0;
  double headingVisionChange = 0;
  double accumulatedVisionHeadingError = 0;
  double totalHeadingCorrection;

  long exitConditionPasses = 0; // number of times the PID caused the robot to go over the target distance
  DriveSpeedConfig searchSpeedConfig;
  int guideType = VISION_GUIDE;
  int preGuideType = guideType;
  bool timeout_exit = false;
  bool search_found_goal = false;

  set_drive_motor_speed(spinConfig, robot);
  while((timeout_exit == false) && (search_found_goal == false)) { //search
    if(pros::millis() > endTimeMillis) {
      timeout_exit = true;
      set_drive_motor_speed_zero(robot);
      break;
    }

    currentInertialHeading = get_robot_heading_lib(robot);
    if(searchDirection == CLOCKWISE){
      if(currentInertialHeading <= maxTargetInertialHeading){
        search_found_goal = true;
        set_drive_motor_speed_zero(robot);
        break;
      }
    } else { //COUNTER_CLOCKWISE
      if(currentInertialHeading >= maxTargetInertialHeading){
        search_found_goal = true;
        set_drive_motor_speed_zero(robot);
        break;
      }
    }

    closest_goal = front_vision.get_by_sig(0, goal_color_signature);
    pros::lcd::print(0, "%d", closest_goal.width);

    if(closest_goal.width > goal_min_width && abs(closest_goal.width) < 316){
      inertialGuideTarget = get_robot_heading_lib(robot);
      set_drive_motor_speed_zero(robot);
      break;
    }
//    } else {
//      display_vision_error(1, "msg");
//    }
//    number_of_object = front_vision.read_by_sig(0, 3, 2, object_arr);
//    closest_goal = front_vision.get_by_sig(0, DETECT_YELLOW_GOAL_SIG);
//    if((errno == EAGAIN) || (errno == EINVAL) || (errno == EDOM) || (errno == EAGAIN)) {
//    if(number_of_object > 0) { //find target goal with specified color
//      pros::lcd::print(0, "%d, %d", object_arr[0].width, errno);
//    } else {
//      pros::lcd::print(0, "camara disconnected");
//    }
/*
      if(closest_goal.width >= goal_min_width){ //find target goal big enough
        pros::lcd::print(1, "Enter 3");
        inertialGuideTarget = get_robot_heading_lib(robot);
        set_drive_motor_speed_zero(robot);
        break;
      }
//    }
*/
    pros::delay(10);
  } // end of first while loop

//waitForTouch();


  while(timeout_exit == false && search_found_goal == false) {
    if(pros::millis() > endTimeMillis){
      timeout_exit = true;
      set_drive_motor_speed_zero(robot);
      break;
    }
    if(exitConditionPasses >= exitConditionExpectedPasses){ // this will determine the number of times you want the bot to cross the target line
      set_drive_motor_speed_zero(robot);
      break;
    }

    guideType = INERTIAL_GUIDE;
    closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
    if(closest_goal.width < goal_min_width || abs(closest_goal.width) < 316){
      guideType = VISION_GUIDE;
    } else {
      guideType = VISION_GUIDE;
    }

    if(guideType == INERTIAL_GUIDE){
      pros::lcd::print(1, "INERTIAL_GUIDE");
      currentInertialHeading = get_robot_heading_lib(robot);
      currentInertialHeadingError = inertialGuideTarget - currentInertialHeading; //finds the diff between current and target heading
      if(preGuideType == VISION_GUIDE){
        priorInertialHeading = currentInertialHeading;
        priorInertialHeadingError = currentInertialHeadingError;
      }
      accumulatedInertialHeadingError = currentInertialHeadingError + accumulatedInertialHeadingError;
      headingInertialChange = currentInertialHeading - priorInertialHeading;
      //all
      totalHeadingCorrection = currentInertialHeadingError * inertialKP
                               + headingInertialChange * inertialKD
                               + accumulatedInertialHeadingError * inertialKI;

      priorInertialHeading = currentInertialHeading; //resets the heading for next loop
      if(currentInertialHeadingError == 0){
        exitConditionPasses ++;
      } else if(priorInertialHeadingError > 0 && currentInertialHeadingError < 0){  // uses error to tell how close the robot is to the target
        exitConditionPasses ++;
      } else if(priorInertialHeadingError < 0 && currentInertialHeadingError > 0){
        exitConditionPasses ++;
      }
      priorInertialHeadingError = currentInertialHeadingError;

    } else { //VISION_GUIDE
      pros::lcd::print(1, "VISION_GUIDE");
      if (closest_goal.x_middle_coord >= 156 && closest_goal.x_middle_coord <= 160){
        inertialGuideTarget = get_robot_heading_lib(robot);
      }
      currentVisionHeading = closest_goal.x_middle_coord;
      currentVisionHeadingError = targetVisionHeading - currentVisionHeading;//finds the diff between current and target heading
      if(preGuideType == INERTIAL_GUIDE){
        priorVisionHeading = currentVisionHeading;
        priorVisionHeadingError = currentVisionHeadingError;
      }
      //Integral
      accumulatedVisionHeadingError = currentVisionHeadingError + accumulatedVisionHeadingError;
      //Derivative
      headingVisionChange = currentVisionHeading - priorVisionHeading;
      //all
      totalHeadingCorrection = currentVisionHeadingError * visionKP
                             + accumulatedVisionHeadingError * visionKI
                             + headingVisionChange * visionKD;
      priorVisionHeading = currentVisionHeading; //resets the heading for next loop
      if(currentVisionHeadingError == 0){
        exitConditionPasses ++;
      } else if(priorVisionHeadingError > 0 && currentVisionHeadingError < 0){  // uses error to tell how close the robot is to the target
        exitConditionPasses ++;
      } else if(priorVisionHeadingError < 0 && currentVisionHeadingError > 0){
        exitConditionPasses ++;
      }
      priorVisionHeadingError = currentVisionHeadingError;
    }

    totalHeadingCorrection = truncateNumber(totalHeadingCorrection, maxPower);
    searchSpeedConfig = assignDriveSpeed(0 - totalHeadingCorrection, totalHeadingCorrection,
                                         0 - totalHeadingCorrection, totalHeadingCorrection);
    set_drive_motor_speed(searchSpeedConfig, robot);
    preGuideType = guideType;
    pros::delay(5); // the targeted delay time needs to be consistent every loop by finding how long the execution within the loop took and then subtracting that from the preferred delay time
  }

  set_drive_motor_speed_zero(robot);
}



bool combine_two_goals(pros::vision_object_s_t *first_goal, pros::vision_object_s_t *second_goal){
  pros::vision_object_s_t left_goal;
  pros::vision_object_s_t right_goal;
  if(first_goal[0].left_coord < second_goal[0].left_coord) {
    left_goal = first_goal[0];
    right_goal = second_goal[0];
  } else {
    left_goal = second_goal[0];
    right_goal = first_goal[0];
  }

  if(left_goal.width < 100 && right_goal.width < 60) {
    return false;
  }

  if(abs(left_goal.height - right_goal.height) > 10) {
    return false;
  }

  if(right_goal.left_coord - (left_goal.left_coord + left_goal.width) > 22) {
    return false;
  }

//  first_goal[0].width = right_goal.left_coord - left_goal.left_coord + right_goal.width;
  first_goal[0].width = left_goal.width * 2;
  return true;
}

/***************************************************************************************************
* ball_color_signature must be assigned as one of RED_BALL_SIG or BLUE_BALL_SIG
****************************************************************************************************/
void new_goStraightCm_Front_Vision(double cmDistance, double robotInertialHeadingLib, int maxSpeed,
                                   int goal_color_signature, Vision vision_sensor,
                                   double headingKP, double headingKI, double headingKD,
                                   double distanceKP, double distanceKI, double distanceKD,
                                   double visionKP, double visionKI, double visionKD,
                                   long timeoutMili, int exitConditionExpectedPasses, Hardware robot){

  if(goal_color_signature != DETECT_RED_GOAL_SIG && goal_color_signature != DETECT_BLUE_GOAL_SIG
     && goal_color_signature != DETECT_YELLOW_GOAL_SIG) {
    return;
  }

  pros::vision_object_s_t closestGoal = vision_sensor.get_by_sig(0, goal_color_signature);

  int guideType = VISION_GUIDE;
  int preGuideType = VISION_GUIDE;

  if(errno == EAGAIN || errno == EINVAL || errno == EDOM || errno == EAGAIN){
    guideType = INERTIAL_GUIDE;
    preGuideType = INERTIAL_GUIDE;
    pros::lcd::print(1, "inertial guide ");
  } else{
    //vision guide, does nothing
    pros::lcd::print(1, "vision guide ");
  }

  long startingTime = pros::millis();
  long endTime = startingTime + timeoutMili;
  long enterTime;
  long currentTime;
  long previousTime = 0;
  long substitutedTime;
  long delayTime = 10;

  cmDistance = std::abs(cmDistance);
  double currentEncoder = Y_encoder.get_value();
  double targetEncoderDegrees = cmDistance/(6.985*3.14) * 360 + currentEncoder;
  double currentEncoderError = 0;
  double previousEncoderError = 0;
  double accumulatedEncoderError = 0;
  double distanceProportionalCorrection = 0;
  double distanceIntegralCorrection = 0;
  double distanceDerivativeCorrection = 0;
  double totalDistanceCorrection = 0;

  double targetVisionHeading = 0;
  double currentVisionHeading = closestGoal.left_coord;
  double currentVisionHeadingError = 0;
  double priorVisionHeading = currentVisionHeading;
  double headingVisionChange = 0;
  double accumulatedVisionHeadingError = 0;

  double currentInertialHeading = get_robot_heading_lib(robot);
  double targetInertialHeading = convert_target_to_relative_angle_lib(currentInertialHeading, robotInertialHeadingLib);
  double currentInertialHeadingError = 0;
  double priorInertialHeading = currentVisionHeading;
  double headingInertialChange = 0;
  double accumulatedInertialHeadingError = 0;

  double headingProportionalCorrection = 0;
  double headingIntegralCorrection = 0;
  double headingDerivativeCorrection = 0;
  double totalHeadingCorrection = 0;

  long exitConditionPasses = 0; // number of times the PID caused the robot to go over the target distance
  DriveSpeedConfig distanceSpeedConfig;
  DriveSpeedConfig headingSpeedConfig;
  DriveSpeedConfig totalSpeedConfig;

  long n = 0;
  long num_gyro = 0;
  long num_vision = 0;
  int target_left = 0;

  while(true){
      enterTime = pros::millis();
      closestGoal.width = -10;
//      closest_goal = vision_sensor.get_by_sig(0, goal_color_signature);
      closestGoal = vision_sensor.get_by_sig(0, goal_color_signature);
      target_left = get_left_coordinate(closestGoal.width);
      targetVisionHeading = target_left;
//      pros::lcd::print(1, "w = %d%", closest_goal.width);
      if(errno == EAGAIN){      //|| errno == ENODEV){  // read vision sensor error
        n++;
        preGuideType = guideType;
        guideType = INERTIAL_GUIDE;
        pros::lcd::print(2, "ERR: no vision %d %d %d", errno, ENODEV, n);
      } else { //read vision sensor successfully
        if (closestGoal.width >= 20 && closestGoal.left_coord >= target_left - 2
                                           && closestGoal.left_coord <= target_left + 2){
          targetInertialHeading = get_robot_heading_lib(robot);
          n++;
        }

        if (closestGoal.x_middle_coord <= 10 || closestGoal.x_middle_coord >= 306
                                                    || closestGoal.width <= 20){
          if(guideType == VISION_GUIDE){
            guideType = INERTIAL_GUIDE;   //inertial sensor guide
          }
        } else {
          if(guideType == INERTIAL_GUIDE){
            guideType = VISION_GUIDE;   //vision sensor guide
          }
        }

        if (closestGoal.width > 240){
          guideType = INERTIAL_GUIDE;
          targetInertialHeading = get_robot_heading_lib(robot);
        }
      }

      currentEncoder = Y_encoder.get_value();
      currentEncoderError = targetEncoderDegrees - currentEncoder;
      if(currentEncoderError == 0){
        exitConditionPasses ++;
      }
      else if(previousEncoderError > 0 && currentEncoderError < 0){  // uses error to tell how close the robot is to the target
        exitConditionPasses ++;
      }
      else if(currentEncoderError > 0 && previousEncoderError < 0){
        exitConditionPasses ++;
      }

      if(exitConditionPasses >= exitConditionExpectedPasses){ // this will determine the number of times you want the bot to cross the target line
        set_drive_motor_speed_zero(robot);
        break;
      } else if(pros::millis() >= endTime){
        set_drive_motor_speed_zero(robot);
        break;
      }

      //HEADING CORRECTION/////////////////////////////////////////////////////////////////////////////////////////
      if(guideType == INERTIAL_GUIDE){
        num_gyro++;
        currentInertialHeading = get_robot_heading_lib(robot);
        if(preGuideType == VISION_GUIDE){
          priorInertialHeading = currentInertialHeading;
        }
        currentInertialHeadingError = targetInertialHeading - currentInertialHeading; //finds the diff between current and target heading
        headingProportionalCorrection = currentInertialHeadingError * headingKP;

        accumulatedInertialHeadingError = currentInertialHeadingError + accumulatedInertialHeadingError;
        headingIntegralCorrection = accumulatedInertialHeadingError * headingKI;

        headingInertialChange = currentInertialHeading - priorInertialHeading;
        headingDerivativeCorrection = headingInertialChange * headingKD;
        //all
        totalHeadingCorrection = (headingProportionalCorrection + headingIntegralCorrection +
                                      headingDerivativeCorrection);
        priorInertialHeading = currentInertialHeading; //resets the heading for next loop
      } else if (guideType == VISION_GUIDE){
        num_vision++;
        currentVisionHeading = closestGoal.left_coord;
        if(preGuideType == INERTIAL_GUIDE){
          priorVisionHeading = currentVisionHeading;
        }

        currentVisionHeadingError =  targetVisionHeading - currentVisionHeading;//finds the diff between current and target heading
        headingProportionalCorrection = currentVisionHeadingError * visionKP;
        //Integral
        accumulatedVisionHeadingError = currentVisionHeadingError + accumulatedVisionHeadingError;
        headingIntegralCorrection = accumulatedVisionHeadingError * visionKI;
        //Derivative
        headingVisionChange = currentVisionHeading - priorVisionHeading;
        headingDerivativeCorrection = headingVisionChange * visionKD;
        //all
        totalHeadingCorrection = headingProportionalCorrection + headingIntegralCorrection + headingDerivativeCorrection;
        priorVisionHeading = currentVisionHeading; //resets the heading for next loop
      }

      totalHeadingCorrection = truncateNumber(totalHeadingCorrection, maxSpeed/2);

      //DISTANCE CORRECTION////////////////////////////////////////////////////////////////////////////////////////////////////
      distanceProportionalCorrection = currentEncoderError * distanceKP; // simple proportional calculation
      accumulatedEncoderError = accumulatedEncoderError + currentEncoderError;
      distanceIntegralCorrection = accumulatedEncoderError * distanceKI * delayTime;
      distanceDerivativeCorrection = (currentEncoderError - previousEncoderError) * distanceKD / delayTime; // error difference in 2 measurements * the kD.
      totalDistanceCorrection = distanceProportionalCorrection + distanceIntegralCorrection + distanceDerivativeCorrection; // sum of the 2 corrections
      totalDistanceCorrection = truncateNumber(totalDistanceCorrection, maxSpeed);

      headingSpeedConfig  = assignDriveSpeed(0 - totalHeadingCorrection, totalHeadingCorrection,
                                             0 - totalHeadingCorrection, totalHeadingCorrection);
      distanceSpeedConfig  = assignDriveSpeed(totalDistanceCorrection, totalDistanceCorrection,
                                              totalDistanceCorrection, totalDistanceCorrection);
      totalSpeedConfig = proportional_projection_two_speed_config(distanceSpeedConfig, headingSpeedConfig, maxSpeed);
      set_drive_motor_speed(totalSpeedConfig, robot);

      preGuideType = guideType;
      previousEncoderError = currentEncoderError;
      previousTime = currentTime;
      substitutedTime = pros::millis() - enterTime;
      pros::delay(delayTime - substitutedTime); // the targeted delay time needs to be consistent every loop by finding how long the execution within the loop took and then subtracting that from the preferred delay time
    }
}
