package frc.robot.Utilities.Drivers;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Utils;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class MinoDoubleSol implements Constants {

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
		}
	}
	
	public void toggle() {
		
		if(!wasToggled) {
			new Thread( () -> {
                wasToggled = true;
                sol.set(sol.get() == Value.kForward ? Value.kReverse : Value.kForward);
                Utils.sleep(1500);
                wasToggled = false;
			}).start();
		}
		
	}
	
	public Value getValue() {
		return sol.get();
	}

	
	public void reset() {
		wasToggled = false;
	}

}
