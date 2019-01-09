package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Utils;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class MinoDoubleSol extends Subsystem implements Constants {

	private final DoubleSolenoid sol;
	
	private long prevToggleTime;
	
	private boolean wasToggled;
	
	public MinoDoubleSol(int forwardChannel, int reverseChannel) {
		sol = new DoubleSolenoid(forwardChannel, reverseChannel);
		prevToggleTime = System.nanoTime();
		wasToggled = false;
	}
	
	public boolean safetyTimeoutCleared() {
		if ((System.nanoTime() - prevToggleTime) > 1.5E9) {
			prevToggleTime = System.nanoTime();
			return true;
		}
		return false;
	}
	
	public void set(Value value) {
		if (sol.get() != value && safetyTimeoutCleared()) {
			sol.set(value);
			smartDashboard();
		}
	}
	
	public void toggle() {
		
		if(!wasToggled) {
			new Thread(new Runnable() {
				public void run() {
					wasToggled = true;
					sol.set(sol.get() == Value.kForward ? Value.kReverse : Value.kForward);
					smartDashboard();
					Utils.sleep(2000);
					wasToggled = false;
				}
			}).start();
		}
		
		//if (safetyTimeoutCleared()) {}
	}
	
	public Value getValue() {return sol.get();}
	
	public abstract void smartDashboard();
	
	public abstract void teleop(Joystick gamepad);
	
	public void reset() {
		wasToggled = false;
	}
	
	@Override
	protected void initDefaultCommand() {
		System.out.println("Minotaur Double Solenoid");
	}

}
