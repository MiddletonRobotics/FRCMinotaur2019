package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.DriveTrain;

public class DropSwitchRight extends Auto {

    private boolean isReady = false;

    @Override
    public void auto() {
        Utils.resetRobot();
        //  Robot.driveTrain.driveVelocity(800, -800);
        try {

            Robot.driveTrain.moveByGyroDistance(168, DriveTrain.Direction.BACKWARD, 0x1.999999999999ap-2, 0x3, 10000);
            Robot.driveTrain.stopDrive();
            if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {

                Robot.driveTrain.turnP(90, DriveTrain.Direction.COUNTERCLOCKWISE, 0.6, 5, 2500);
                Robot.driveTrain.stopDrive();
                Robot.lift.setSetpoint(0.507720410556595 );
                isReady = true;
                Robot.intakeTilt.setSetpoint(.633);
            }else{
                Robot.driveTrain.moveByGyroDistance(60, DriveTrain.Direction.FORWARD, 0x1.999999999999ap-2, 0x3, 10000);
                Robot.driveTrain.stopDrive();
                Robot.driveTrain.turnP(90, DriveTrain.Direction.COUNTERCLOCKWISE, 0.6, 5, 2500);
                Robot.driveTrain.stopDrive();
                Robot.driveTrain.moveByGyroDistance(220, DriveTrain.Direction.FORWARD, 0x1.999999999999ap-2, 0x3, 10000);
                Robot.driveTrain.stopDrive();
                Robot.driveTrain.turnP(90, DriveTrain.Direction.COUNTERCLOCKWISE, 0.6, 5, 2500);
                Robot.driveTrain.stopDrive();
                Robot.driveTrain.moveByGyroDistance(60, DriveTrain.Direction.FORWARD, 0x1.999999999999ap-2, 0x3, 10000);
                Robot.driveTrain.stopDrive();

                Robot.driveTrain.turnP(90, DriveTrain.Direction.COUNTERCLOCKWISE, 0.6, 5, 2500);
                Robot.driveTrain.stopDrive();
                Robot.driveTrain.moveByGyroDistance(12, DriveTrain.Direction.FORWARD, 0x1.999999999999ap-2, 0x3, 10000);
                Robot.driveTrain.stopDrive();
                Robot.lift.setSetpoint(0.507720410556595 );
                isReady = true;
                Robot.intakeTilt.setSetpoint(.633);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        // Utils.sleep(3500);
        //Robot.driveTrain.stopDrive();

       // Utils.sleep(500);
       // Robot.intake.stop();
    }

    @Override
    public void loop() {
        if (isReady) {
            Robot.lift.autoPID();
            Robot.intakeTilt.stop();
            if(Robot.lift.getPIDController().getError() < 0.08 && Robot.intakeTilt.getPIDController().getError() <  0.004) {
                Robot.intake.outtake();
                Utils.sleep(500);
                Robot.intake.stop();
                isReady = false;
            }
        }


    }

    @Override
    public void stop() {
        Utils.resetRobot();
        isReady = false;

    }
}
