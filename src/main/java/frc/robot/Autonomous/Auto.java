package frc.robot.Autonomous;

import frc.robot.Utilities.Utils;

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