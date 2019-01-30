
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

    public static CameraServer cs;
    public static Joystick gamepad1;
    public static Joystick gamepad2;

    public static DriveTrain driveTrain;
    public static Vision vision;
    public static Lift lift;
    public static Arm arm;


    public static boolean isTeleop = false;
    public static boolean isDisabled = false;

    SendableChooser<Auto> chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        
        gamepad1 = new Joystick(Constants.gamepad1Port);
        gamepad2 = new Joystick(Constants.gamepad2Port);
        cs = CameraServer.getInstance();
        cs.startAutomaticCapture();

        driveTrain = DriveTrain.getInstance();
        vision = new Vision();
        lift = Lift.getInstance();
        arm = Arm.getInstance();


        chooser.setDefaultOption("Do nothing :O", null);

//        chooser.addObject("uWu", new DoubleSwitchMiddleAutoBoi());
        SmartDashboard.putNumber("Delay MS (C H A N G E T H I S E V E R Y M A T C H)", 0);
        SmartDashboard.putData(chooser);

        Utils.resetRobot();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        Robot.isTeleop = false;
        Robot.isDisabled = true;
        SmartDashboard.putNumber("Disabled Init Ran", 1);
    }

    @Override
    public void disabledPeriodic() {
        Robot.isDisabled = true;
        SmartDashboard.putNumber("Disabled Init Ran", 2);
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */

    //Hey change this
    private Auto a = new TurnTest();


    @Override
    public void autonomousInit() {
        Robot.isDisabled = false;
        SmartDashboard.putNumber("Disabled Init Ran", 0);

        //EY bruh fix this
        //a = chooser.getSelected();

        if (a != null) {
            a.auto();
        }
    }



    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        
        Scheduler.getInstance().run();
        if (a != null) {
            a.loop();
        }

    }

    @Override
    public void teleopInit() {

        if (a != null) {
            a.threadLock = null;
            a.stop();
            a = null;
        }
        isTeleop = true;
        Utils.resetRobot();

        isToggled = false;
    }

    boolean isToggled = false;

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        driveTrain.teleop(gamepad1);
        lift.teleop(gamepad2);
        arm.teleop(gamepad1);
        vision.teleop(gamepad1);




        //SmartDashboard.putNumber("Potentiometer:", lift.potentiometer.pidGet());
//		SmartDashboard.putNumber("Distance form egeg", rangeSensor.getDistance());
       /* SmartDashboard.putNumber("Left motor", driveTrain.getLeftTalon().get());
        SmartDashboard.putNumber("Right motor", driveTrain.getRightTalon().get());

        SmartDashboard.putNumber("GyroAngle", driveTrain.getGyroAngle());
        SmartDashboard.putNumber("L RPM", driveTrain.getLeftTalon().getSelectedSensorVelocity(0) >= SmartDashboard.getNumber("L RPM", 0) ? driveTrain.getLeftTalon().getSelectedSensorVelocity(0) : SmartDashboard.getNumber("L RPM", 0));
        SmartDashboard.putNumber("R RPM", driveTrain.getRightTalon().getSelectedSensorVelocity(0) >= SmartDashboard.getNumber("R RPM", 0) ? driveTrain.getRightTalon().getSelectedSensorVelocity(0) : SmartDashboard.getNumber("R RPM", 0));
        SmartDashboard.putNumber("LCurrent", driveTrain.getLeftTalon().getOutputCurrent());
        SmartDashboard.putNumber("RCurrent", driveTrain.getRightTalon().getOutputCurrent());*/

    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {

    }

}
