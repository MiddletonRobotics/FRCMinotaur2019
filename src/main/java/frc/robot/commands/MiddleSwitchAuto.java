package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.AutoInterruptedException;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.DriveTrain;

public class MiddleSwitchAuto extends Auto implements Constants {
    char side;

    @Override
    public void auto() {
        Utils.resetRobot();

        side = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
        Robot.driveTrain.setProfile(2);

        Robot.driveTrain.driveDistance(-13);
        Robot.lift.setSetpoint(Utils.iPhoneMath(.24));
        Robot.intakeTilt.setSetpoint(TILT_TOP);
        Utils.sleep(1000);

        if (side == 'L') {
            try {
                Robot.driveTrain.turnP(35, DriveTrain.Direction.COUNTERCLOCKWISE, 0.5, 0.5, 2500, true);
                long sTime = System.currentTimeMillis();
                do {
                    Robot.intakeTilt.stop();
                    Robot.lift.autoPID();
                }
                while (System.currentTimeMillis() - sTime < 2500 && Robot.lift.getPIDController().getError() > 0.08 && Robot.intakeTilt.getPIDController().getError() > 0.004);

                Robot.driveTrain.driveDistance(-90);
               Utils.sleep(2500);
                Robot.driveTrain.driveDistance(-10);
                Utils.sleep(500);


                Robot.intakeTilt.setSetpoint(TILT_MIDDLE);
                sTime = System.currentTimeMillis();
                do {
                    Robot.intakeTilt.stop();
                    Robot.lift.autoPID();
                }
                while (System.currentTimeMillis() - sTime < 2500 && (Robot.lift.getPIDController().getError() > 0.08 || Robot.intakeTilt.getPIDController().getError() > 0.004));
                Robot.intake.outtake();
                Utils.sleep(1000);
                Robot.intake.stop();
            } catch (AutoInterruptedException e) {
                stop();
            }
        } else if (side == 'R') {
            try {
                Robot.driveTrain.turnP(35, DriveTrain.Direction.CLOCKWISE, 0.5, 0.5, 2500, true);
                long sTime = System.currentTimeMillis();
                do {
                    Robot.intakeTilt.stop();
                    Robot.lift.autoPID();
                }
                while (System.currentTimeMillis() - sTime < 2500 && Robot.lift.getPIDController().getError() > 0.08 && Robot.intakeTilt.getPIDController().getError() > 0.004);

                Robot.driveTrain.driveDistance(-90);

                Utils.sleep(2500);
                Robot.driveTrain.driveDistance(-10);
                Utils.sleep(500);
                Robot.intakeTilt.setSetpoint(TILT_MIDDLE);
                sTime = System.currentTimeMillis();
                do {
                    Robot.intakeTilt.stop();
                    Robot.lift.autoPID();
                }
                while (System.currentTimeMillis() - sTime < 2500 && (Robot.lift.getPIDController().getError() > 0.08 || Robot.intakeTilt.getPIDController().getError() > 0.004));

                Robot.intake.outtake();
                Utils.sleep(1000);
                Robot.intake.stop();
            } catch (AutoInterruptedException e) {
                stop();
            }
        }


    }

    @Override
    public void loop() {
        Robot.intakeTilt.stop();
        Robot.lift.autoPID();

    }

    @Override
    public void stop() {
        Utils.resetRobot();
        Robot.driveTrain.setProfile(0);
    }
}
