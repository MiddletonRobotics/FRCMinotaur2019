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

        talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        talon.setSensorPhase(reversed);
        // master.configClosedloopRamp(0.25, 0);
        talon.configOpenloopRamp(0.25, 0);
//      masteqr.configNominalOutputForward(NOMINAL_OUTPUT_VOLTAGE, -NOMINAL_OUTPUT_VOLTAGE);
        //    master.configPeakOutputVoltage(+PEAK_OUTPUT_VOLTAGE, -PEAK_OUTPUT_VOLTAGE);

        //aster.configSen R);

        talon.configAllowableClosedloopError(0, 0, 0);

        //  master.setVoltageRampRate(48);
/*        talon.config_kF(0, kfDriveTrainVel, 0);
        talon.config_kP(0, kpDriveTrainVel, 0);
        talon.config_kI(0, kiDriveTrainVel, 0);
        talon.config_kD(0, kdDriveTrainVel, 0);

        talon.config_kF(1, kfDriveTrainPos, 0);
        talon.config_kP(1, kpDriveTrainPos, 0);
        talon.config_kI(1, kiDriveTrainPos, 0);
        talon.config_kD(1, kdDriveTrainPos, 0);

        talon.config_kF(2, kfDriveTrainPos2, 0);
        talon.config_kP(2, kpDriveTrainPos2, 0);
        talon.config_kI(2, kiDriveTrainPos2, 0);
        talon.config_kD(2, kdDriveTrainPos2, 0);*/
        //aster.setF(kfDriveTrainVbus);
        //master.setP(kpDriveTrainVbus);
        //master.setI(kiDriveTrainVbus);.config
        //     master.setD(kdDriveTrainVbus);
    }
}
