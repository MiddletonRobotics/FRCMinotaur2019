package frc.robot.commands;

import frc.robot.Utils;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;

public abstract class Auto {

	public Thread thread;
	public Object threadLock;
	public int delay;

	
	public Auto() {}

	public void start(){
		Utils.sleep(delay);
	}
	
	public abstract void auto();
	public abstract void loop();
	public abstract void stop();
	
}