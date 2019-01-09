package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Ramps extends Subsystem implements Constants, Section {
    @Override
    protected void initDefaultCommand() {
        System.out.println("Minotaur Ramps");
    }

    @Override
    public void teleop(Joystick gamepad) {
        /*Idk how this will work but it will probably be like scale mode in 2016 so basically

          Fritz lines up the robot first then puts it in ramp mode
          by pressing the back button or start button something like that
          **********************************************************************

          boolean rampsDropped

          if in ramp mode {

            disable driveTrain, lift, and grabber

            if A button pressed && !rampsDropped
                actuate pneumatics/servo that causes ramps to drop

            if rampsDropped {
                if left button pressed
                     lift left ramp

                if right button pressed
                    lift right ramp
            }
         }
         */
    }

    @Override
    public void reset() {
        //Put ramps in original position
        //Put latch that drops ramps back in original position
    }
}
