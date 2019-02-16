package frc.robot.Autonomous;

import frc.robot.Autonomous.StateMachine.StateMachineRunner;
import frc.robot.Utilities.Utils;

public abstract class Auto {

	public Thread thread;
	public Object threadLock;
	public int delay;
	public StateMachineRunner stateMachine = new StateMachineRunner(200);

	public Auto() {}

	public void start(){
		Utils.sleep(delay);
	}
	
	public abstract void auto();
	public abstract void loop();
	public abstract void stop();
	
}