package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Utils;

public class MoveAutoLine extends Auto {
    @Override
    public void auto() {
        Utils.resetRobot();
//        Robot.driveTrain.setProfile(1);
        Robot.driveTrain.driveVelocity(800, -800);


        Utils.sleep(2500);
//                Robot.driveTrain.driveDistance(264);

        Robot.driveTrain.stopDrive();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        Robot.driveTrain.stopDrive();
    }
}
