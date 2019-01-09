package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

public interface Section {
	
	void teleop(Joystick gamepad);
	void reset();

}
