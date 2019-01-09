package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface Constants {

    double VISION_TARG_WIDTH = 10;
    //Hardware Ports
    int masterLeftPort = 5;
    int slaveLeftPort = 6;
    int masterRightPort = 3;
    int slaveRightPort = 4;

    int leftIntakePort = 1;
    int rightIntakePort = 2;

    int tiltMotorPort = 0;

    int liftMotor1Port = 49;
    int liftMotor2Port = 39;
    int liftMotor3Port = 29;
    int liftMotor4Port = 19;

    int winchMotor1Port = 3;
    int winchMotor2Port = 4;

    int limitSwitchTiltBottomPort = 2;
    int limitSwitchTiltTopPort = 3;

    int limitSwitchLiftBottomPort = 0;
    int limitSwitchLiftTopPort = 1;

    int intakeTiltPotPort = 0;

    SPI.Port gyroPort = SPI.Port.kOnboardCS0;

    int gamepad1Port = 0;
    int gamepad2Port = 1;

    //Gamepad Axes and Buttons
    int LEFT_X_AXIS = 0;
    int LEFT_Y_AXIS = 1;
    int LEFT_T_AXIS = 2;
    int RIGHT_T_AXIS = 3;
    int RIGHT_X_AXIS = 4;
    int RIGHT_Y_AXIS = 5;

    int BTN_A = 1;
    int BTN_B = 2;
    int BTN_X = 3;
    int BTN_Y = 4;
    int BTN_LB = 5;
    int BTN_RB = 6;
    int BTN_BACK = 7;
    int BTN_START = 8;
    int BTN_LEFT_JOYSTICK = 9;
    int BTN_RIGHT_JOYSTICK = 10;

    int DPAD_NOT_PRESSED = -1;

    int DPAD_UP = 0;
    int DPAD_RIGHT = 90;
    int DPAD_DOWN = 180;
    int DPAD_LEFT = 270;

    int DPAD_NORTH = 0;
    int DPAD_NORTHEAST = 45;
    int DPAD_EAST = 90;
    int DPAD_SOUTHEAST = 135;
    int DPAD_SOUTH = 180;
    int DPAD_SOUTHWEST = 225;
    int DPAD_WEST = 270;
    int DPAD_NORTHWEST = 315;


    double TILT_BOTTOM = .53;
    double TILT_MIDDLE = .73;
    double TILT_TOP = .893;
    double TILT_BACK = .893;


    double LIFT_TOP = 0.0106;
    double LIFT_BOTTOM = .615;

    double SENT_FROM_MY_iPHONE = LIFT_TOP - LIFT_BOTTOM;

    double kfDriveTrainVel = .687;
    double kpDriveTrainVel = 0.0842;
    double kiDriveTrainVel = 0.0;
    double kdDriveTrainVel = 0;

    double kfDriveTrainPos = 0.05;
    double kpDriveTrainPos = 0.08;
    double kiDriveTrainPos = 0.0;
    double kdDriveTrainPos = 0;

    double kfDriveTrainPos2 = 0.01;
    double kpDriveTrainPos2 = 0.454000254;
    double kiDriveTrainPos2 = 0.0;
    double kdDriveTrainPos2 = 0;


    int maxRPM = 1250;

    float NOMINAL_OUTPUT_VOLTAGE = 0.0f;
    float PEAK_OUTPUT_VOLTAGE = 12.0f;
    int ANALOG_OUTPUT_VOLTAGE = 5;

    int RANGE_VOLTAGE_CONSTANT = 5021;

    int NATIVE_PER_ROTATION = 1024;
    int ENCODER_PPR = 256;
    double WHEEL_DIAMETER = 4.0;
    double CLICKS_PER_INCH = (ENCODER_PPR * 4) / (WHEEL_DIAMETER * Math.PI);
    double VOLTS_PER_MM = (ANALOG_OUTPUT_VOLTAGE / RANGE_VOLTAGE_CONSTANT);
}
