package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class MinoRangeSensor extends AnalogInput implements Constants {

	public MinoRangeSensor(int channel) {
		super(channel);
	}

	public double getDistance() {
		double inches = (getMillimetres() / 25.4) - 66.5;
		return inches > 0 ? inches : 0;
	}

	public double getMillimetres() {
		return getVoltage() / .000996;// VOLTS_PER_MM;
	}

}
