package frc.robot.Utilities.Drivers;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Utilities.Constants.Constants;

public class MinoGamepad extends Joystick implements Constants {
    /**
     * Construct an instance of a joystick. The joystick index is the USB port on the drivers
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public MinoGamepad(int port) {
        super(port);
    }
    
    
    public boolean a() {
        return getRawButton(BTN_A);
    }    
    
    public boolean b() {
        return getRawButton(BTN_B);
    }    

    public boolean x() {
        return getRawButton(BTN_X);
    }    
    
    public boolean y() {
        return getRawButton(BTN_Y);
    }    
    
    public boolean leftBumper() {
        return getRawButton(BTN_LB);
    }    
    
    public boolean rightBumper() {
        return getRawButton(BTN_RB);
    }    
    
    public boolean back() {
        return getRawButton(BTN_BACK);
    }    
    
    public boolean start() {
        return getRawButton(BTN_START);
    }    
    
    public boolean leftStickButton() {
        return getRawButton(BTN_LEFT_JOYSTICK);
    }    
    
    public boolean rightStickButton() {
        return getRawButton(BTN_RIGHT_JOYSTICK);
    }

    public boolean dpadUp() {
        return getPOV() == DPAD_NORTH;
    }

    public boolean dpadDown() {
        return getPOV() == DPAD_SOUTH;
    }

    public boolean dpadRight() {
        return getPOV() == DPAD_EAST;
    }

    public boolean dpadLeft() {
        return getPOV() == DPAD_WEST;
    }

    public boolean dpadTopRight() {
        return getPOV() == DPAD_NORTHEAST;
    }

    public boolean dpadTopLeft() {
        return getPOV() == DPAD_NORTHWEST;
    }

    public boolean dpadBottomRight() {
        return getPOV() == DPAD_SOUTHEAST;
    }

    public boolean dpadBottomLeft() {
        return getPOV() == DPAD_SOUTHWEST;
    }




    public double leftStickX() {
        return getRawAxis(LEFT_X_AXIS);
    }

    public double leftStickY() {
        return getRawAxis(LEFT_Y_AXIS);
    }

    public double rightStickX() {
        return getRawAxis(RIGHT_X_AXIS);
    }

    public double rightStickY() {
        return getRawAxis(RIGHT_Y_AXIS);
    }

    public double leftTrigger() {
        return getRawAxis(LEFT_T_AXIS);
    }

    public double rightTrigger() {
        return getRawAxis(RIGHT_T_AXIS);
    }

    public boolean rightTriggerPressed() {
        return getRawAxis(RIGHT_T_AXIS) > 0.1 ? true: false;
    }

    public boolean leftTriggerPressed() {
        return getRawAxis(LEFT_T_AXIS) > 0.1 ? true: false;
    }

}
