package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Intake extends Subsystem implements Constants, Section {
    private VictorSP leftIntakeMotor;
    private VictorSP rightIntakeMotor;
    private double speedLimit = 1;

    public Intake() {
        leftIntakeMotor = new VictorSP(leftIntakePort);
        rightIntakeMotor = new VictorSP(rightIntakePort);
    }

    @Override
    protected void initDefaultCommand() {
        System.out.println("Mintoaur Intake (Rip Cheenar-Sama 2013-2017)");
    }

    @Override
    public void teleop(Joystick gamepad) {

        if (gamepad.getRawAxis(LEFT_T_AXIS) > 0.1) {
            leftIntakeMotor.set(-gamepad.getRawAxis(LEFT_T_AXIS) * speedLimit);
            rightIntakeMotor.set(gamepad.getRawAxis(LEFT_T_AXIS) * speedLimit);
//            System.out.println(gamepad.getRawAxis(LEFT_T_AXIS)*.5);
        } else if (gamepad.getRawAxis(RIGHT_T_AXIS) > 0.1) {
            leftIntakeMotor.set(gamepad.getRawAxis(RIGHT_T_AXIS));
            rightIntakeMotor.set(-gamepad.getRawAxis(RIGHT_T_AXIS));
//            System.out.println(gamepad.getRawAxis(RIGHT_T_AXIS));
        } else {
            stop();
        }


    }

    public void intake() {
        leftIntakeMotor.set(-1);
        rightIntakeMotor.set(1);
    }

    public void outtake(double d) {
        leftIntakeMotor.set(-d);
        rightIntakeMotor.set(d);
    }

    public void outtake() {
        leftIntakeMotor.set(speedLimit);
        rightIntakeMotor.set(speedLimit);
    }

    public void stop() {
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }

    @Override
    public void reset() {
        stop();
    }

    public void setSpeedLimit(double speedLimit) {
        this.speedLimit = speedLimit;
    }
}
