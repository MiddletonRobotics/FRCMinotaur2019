package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Drivers.CKTalonSRX;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Section;

public class Arm extends Subsystem implements Section, Constants {

    private CKTalonSRX armTalon;

    public CKTalonSRX getArmTalon() {
        return this.armTalon;
    }

    private static Arm instance = null;


    private Arm() {
        armTalon = new CKTalonSRX(armTalonID, armTalonPDPSlot);
        init();
    }

    public static Arm getInstance() {
        if(instance == null) {
            instance = new Arm();
        }

        return instance;
    }

    private void init() {

        armTalon.setInverted(true);
        armTalon.setSensorPhase(true);
        armTalon.setNeutralMode(NeutralMode.Brake);

        armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        //mIntake2Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0) 
        //armTalon.configRemoteFeedbackFilter(kIntake2MotorId, RemoteSensorSource.TalonSRX_SelectedSensor, 0, 0) 
        //armTalon.configRemoteFeedbackFilter(0x00, RemoteSensorSource.Off, 1, 0) 
        //armTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0) 

        armTalon.configContinuousCurrentLimit(kArmMaxContinuousCurrentLimit, 0);
        armTalon.configPeakCurrentLimit(kArmMaxPeakCurrentLimit, 0);
        armTalon.configPeakCurrentDuration(kArmMaxPeakCurrentDurationMS, 0);

        armTalon.enableCurrentLimit(true);

        armTalon.configForwardSoftLimitThreshold((int) (kArmSoftMax * kArmEncoderGearRatio * sensorUnitsPerRotationMag), kTimeoutMs);
        armTalon.configReverseSoftLimitThreshold((int) (kArmSoftMin * kArmEncoderGearRatio * sensorUnitsPerRotationMag), kTimeoutMs);
        armTalon.configForwardSoftLimitEnable(true, kTimeoutMs);
        armTalon.configReverseSoftLimitEnable(true, kTimeoutMs);
        armTalon.configAllowableClosedloopError(0, kArmAllowedError, kTimeoutMs);


        armTalon.selectProfileSlot(kArmNormalRateSlot, 0);

        TalonHelper.setPIDGains(armTalon, kArmNormalRateSlot, kArmKp, kArmKi, kArmKd, kArmKf, kArmRampRate, kArmIZone);
        TalonHelper.setPIDGains(armTalon, kArmFastRateSlot, kArmKp, kArmKi, kArmKd, kArmKf, kArmRampRate, kArmIZone);
        TalonHelper.setMotionMagicParams(armTalon, kArmNormalRateSlot, kArmMaxVelocity, kArmMaxAccel);
        TalonHelper.setMotionMagicParams(armTalon, kArmFastRateSlot, kArmMaxVelocity, kArmMaxAccelDownFast);


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
        armTalon.set(ControlMode.PercentOutput, 0);
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
