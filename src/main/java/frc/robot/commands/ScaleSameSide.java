package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.AutoInterruptedException;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.DriveTrain;

public class ScaleSameSide extends Auto implements Constants {

    private char side;

    private boolean stop = false;

    public ScaleSameSide(char c) {
        side = c;
    }

    @Override
    public void auto() {
        stop = false;
        Utils.resetRobot();
        // Robot.driveTrain.driveVelocity(800, -800);
        if (side == 'R') {
            if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'R') {
                try {
                    Robot.driveTrain.driveDistance(-33);
                    Utils.sleep(3000);
                    Robot.driveTrain.stopDrive();
                    Robot.intakeTilt.setSetpoint(TILT_TOP);

                    Robot.driveTrain.turnP(90, DriveTrain.Direction.COUNTERCLOCKWISE, 0.7, 0.5, 1750, true);
                    Robot.driveTrain.driveDistance(6);
                    Utils.sleep(1000);
                    Robot.lift.setSetpoint(Utils.iPhoneMath(.97));
                    long sTime = System.currentTimeMillis();
                    do {
                        Robot.intakeTilt.stop();
                        Robot.lift.autoPID();
//                        System.out.println("W A I T B O I S");
                    } while (System.currentTimeMillis() - sTime < 1500 && !stop);


                    Robot.intakeTilt.setSetpoint(TILT_MIDDLE);
                    sTime = System.currentTimeMillis();
                    do {
                        Robot.intakeTilt.stop();
                        Robot.lift.autoPID();
                    } while (System.currentTimeMillis() - sTime < 750 && !stop);
//Utils.sleep(500);
                    Robot.intake.outtake(.65);
                    Utils.sleep(3000);
                    Robot.intake.stop();
                    Robot.intakeTilt.setSetpoint(TILT_TOP);
                    Robot.lift.setSetpoint(Utils.iPhoneMath(.05));

                } catch (AutoInterruptedException e) {
                    stop();
                    stop = true;
                }
            } else if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
                Robot.driveTrain.driveDistance(-15);
                Utils.sleep(2500);

                try {
                    Robot.driveTrain.turnP(90, DriveTrain.Direction.COUNTERCLOCKWISE, 0.7, 0.5, 2500, true);
                    Robot.driveTrain.driveDistance(-4);
                    Utils.sleep(750);
                } catch (AutoInterruptedException e) {
                    e.printStackTrace();
                }
                Robot.lift.setSetpoint(Utils.iPhoneMath(.27));
                Robot.intakeTilt.setSetpoint(TILT_MIDDLE);

                long sTime = System.currentTimeMillis();
                do {
                    Robot.lift.autoPID();
                    Robot.intakeTilt.stop();
                } while (System.currentTimeMillis() - sTime < 1000);

                Robot.intake.outtake();
                Utils.sleep(3000);
                Robot.intake.stop();


            } else {
                Robot.driveTrain.driveDistance(-24);
                Utils.sleep(2500);
                try {
                    Robot.driveTrain.turnP(90, DriveTrain.Direction.COUNTERCLOCKWISE, 0.7, 0.5, 1750, true);
                } catch (AutoInterruptedException e) {
                    e.printStackTrace();
                }
            }
        } else if (side == 'L') {
            if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'L') {
                try {
                    Robot.driveTrain.driveDistance(-33);
                    Utils.sleep(3000);
                    Robot.driveTrain.stopDrive();
                    Robot.intakeTilt.setSetpoint(TILT_TOP);

                    Robot.driveTrain.turnP(90, DriveTrain.Direction.CLOCKWISE, 0.7, 0.5, 1750, true);
                    Robot.driveTrain.driveDistance(6);
                    Utils.sleep(1000);
                    Robot.lift.setSetpoint(Utils.iPhoneMath(.97));
                    long sTime = System.currentTimeMillis();
                    do {
                        Robot.intakeTilt.stop();
                        Robot.lift.autoPID();
//                        System.out.println("W A I T B O I S");
                    } while (System.currentTimeMillis() - sTime < 1500 && !stop);


                    Robot.intakeTilt.setSetpoint(TILT_MIDDLE);
                    sTime = System.currentTimeMillis();
                    do {
                        Robot.intakeTilt.stop();
                        Robot.lift.autoPID();
                    } while (System.currentTimeMillis() - sTime < 750 && !stop);
//Utils.sleep(500);
                    Robot.intake.outtake(.65);
                    Utils.sleep(3000);
                    Robot.intake.stop();
                    Robot.intakeTilt.setSetpoint(TILT_TOP);
                    Robot.lift.setSetpoint(Utils.iPhoneMath(.05));

                } catch (AutoInterruptedException e) {
                    stop();
                    stop = true;
                }
            } else if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L') {
                Robot.driveTrain.driveDistance(-15);
                Utils.sleep(2500);

                try {
                    Robot.driveTrain.turnP(90, DriveTrain.Direction.CLOCKWISE, 0.7, 0.5, 2500, true);
                    Robot.driveTrain.driveDistance(-4);
                    Utils.sleep(750);
                } catch (AutoInterruptedException e) {
                    e.printStackTrace();
                }
                Robot.lift.setSetpoint(Utils.iPhoneMath(.27));
                Robot.intakeTilt.setSetpoint(TILT_MIDDLE);
                long sTime = System.currentTimeMillis();
                do {
                    Robot.lift.autoPID();
                    Robot.intakeTilt.stop();
                } while (System.currentTimeMillis() - sTime < 1000);
                Robot.intake.outtake();
                Utils.sleep(3000);
                Robot.intake.stop();


            } else {
                Robot.driveTrain.driveDistance(-24);
                Utils.sleep(2500);
                try {
                    Robot.driveTrain.turnP(90, DriveTrain.Direction.CLOCKWISE, 0.7, 0.5, 2500, true);
                } catch (AutoInterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public void loop() {
        if (!stop) {
            Robot.intakeTilt.stop();
            Robot.lift.autoPID();
        }

    }

    @Override
    public void stop() {
        Robot.driveTrain.stopDrive();
    }
}
