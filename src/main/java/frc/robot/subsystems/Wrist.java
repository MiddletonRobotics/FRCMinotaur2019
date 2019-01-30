package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Wrist extends Subsystem implements Section, Constants {

    @Override
    protected void initDefaultCommand() {
        System.out.println("Minotaur Wrist");
    }

    @Override
    public void teleop(Joystick gamepad) {

    }

    @Override
    public void reset() {

    }

}
