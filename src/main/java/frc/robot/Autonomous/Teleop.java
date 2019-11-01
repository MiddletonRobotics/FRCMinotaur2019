package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
import frc.robot.Utilities.Utils;

import static frc.robot.Robot.gamepad1;


public class Teleop extends Auto {
    @Override
    public void auto() {
/*        new Thread( () -> {
            Robot.intake.setPercentSpeed(0.5);
            Utils.sleep(1500);
        }).start();*/
    }

    @Override
    public void loop() {
        Robot.driveTrain.teleop(gamepad1);
//        Robot.lift.teleop(gamepad1);
        Robot.liftPID.teleop(gamepad1);
//        vision.teleop(gamepad1);
        Robot.arm.teleop(gamepad1);
        Robot.intake.teleop(gamepad1);
    }

    @Override
    public void stop() {

    }
}
