package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Arm extends Subsystem implements Section, Constants {

    private WPI_TalonSRX wristTalon;

    public WPI_TalonSRX getWristTalon() {
        return this.wristTalon;
    }

    public Arm() {
        wristTalon = new WPI_TalonSRX(wristTalonPort);
    }

    @Override
    protected void initDefaultCommand() {
        System.out.println("Minotaur Arm");
    }

    @Override
    public void teleop(Joystick gamepad) {

    }

    @Override
    public void reset() {
        wristTalon.set(0);
    }

    private void configTalon(WPI_TalonSRX talon, boolean reversed) {
        talon.set(0);

        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        talon.setSensorPhase(reversed);
/*        talon.configClosedloopRamp(0.25, 0);
        talon.configOpenloopRamp(0.25, 0);*/

        talon.configAllowableClosedloopError(0, 0, 0);
    }
}
