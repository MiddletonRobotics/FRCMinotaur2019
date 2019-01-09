package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Forks extends MinoDoubleSol implements Section, Constants {

    @Override
    protected void initDefaultCommand() {
        System.out.println("sent from my iphone");
    }

    public Forks() {
        super(0, 1);
    }

    @Override
    public void smartDashboard() {
    }

    @Override
    public void teleop(Joystick gamepad) {
        if (gamepad.getPOV() == DPAD_EAST) {
            toggle();
        }
    }

    @Override
    public void reset() {}


}
