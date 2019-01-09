package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;

public class DropSwitchRightNoWhip extends Auto implements Constants {
    boolean isReady;

    @Override
    public void auto() {
        Utils.resetRobot();
        Robot.lift.setSetpoint(Utils.iPhoneMath(.27));
        long sTime = System.currentTimeMillis();
        do {
            Robot.lift.autoPID();
        } while (System.currentTimeMillis() - sTime < 1000);
        Robot.driveTrain.driveVelocity(500, -500);
        Utils.sleep(2250);

        Robot.driveTrain.stopDrive();
        if((DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R')) {
            Robot.intakeTilt.setSetpoint(TILT_BOTTOM);
            isReady = true;
        }
    }

    @Override
    public void loop() {
        Robot.lift.autoPID();
        Robot.intakeTilt.stop();
        if (isReady) {

            if (Robot.lift.getPIDController().getError() < 0.08 && Robot.intakeTilt.getPIDController().getError() < 0.004 && (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R')) {
                Robot.intake.outtake();
                Utils.sleep(1500);
                Robot.intake.stop();
                Robot.intakeTilt.setSetpoint(TILT_TOP);
                isReady = false;
            }
        }
    }

    @Override
    public void stop() {
        Robot.driveTrain.stopDrive();
    }}
