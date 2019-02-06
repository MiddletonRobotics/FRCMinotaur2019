package frc.robot.Utilities;

import frc.robot.Utilities.Drivers.MinoGamepad;

public interface Section {
	
	void teleop(MinoGamepad gamepad);
	void reset();

}
