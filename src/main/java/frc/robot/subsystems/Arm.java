package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Constants.Positions.ArmPositions;
import frc.robot.Utilities.Drivers.CKTalonSRX;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Section;

public class Arm extends Subsystem implements Section, Constants {

    private WPI_TalonSRX armMotor;

    private static Arm instance = null;


    private Arm() {
        armMotor = new WPI_TalonSRX(armTalonID);
        configTalon(armMotor);
    }

    public static Arm getInstance() {
        if(instance == null) {
            instance = new Arm();
        }

        return instance;
    }


    @Override
    protected void initDefaultCommand() {
        System.out.println("Minotaur Arm");
    }

    @Override
    public void teleop(MinoGamepad gamepad) {
/*
        if (!armMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
            resetEncoders();
        }*/

        if (gamepad.y()) {
            armMotor.setSelectedSensorPosition(ArmPositions.armDiscPosition, 0, Constants.kTimeoutMs);

            armMotor.set(ControlMode.MotionMagic, ArmPositions.armDiscPosition);
        }

        //System.out.println(gamepad.leftBumper());
        //System.out.println(/*armMotor.getSensorCollection().getQuadraturePosition()*/armMotor.getMotorOutputPercent());
/*        if (gamepad.rightBumper()) {
            armMotor.set(0.3);
        } else if (gamepad.leftBumper()) {
            armMotor.set(-0.6);
        } else {
            armMotor.set(0);
        }*/
    }

    private void zeroArm() {
        armMotor.set(ControlMode.Disabled, 0);

        armMotor.setSelectedSensorPosition(ArmPositions.armHomePosition, 0, Constants.kTimeoutMs);
        armMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
        armMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        armMotor.set(ControlMode.MotionMagic, ArmPositions.armHomePosition);
    }


    private double degreesToSensorUnits(double degrees) {
        return ((degrees + armDegreeOffset)/360)*sensorUnitsPerRotationMag;
    }

    private double rotationsToSensorUnits(double rotations) {
        return rotations*sensorUnitsPerRotationMag;
    }


    private void setRotationSensorUnits(double sensorUnits) {
        armMotor.set(ControlMode.MotionMagic, sensorUnits);
    }

    private void setRotationDegrees(double degrees) {
        double sensorUnits = degreesToSensorUnits(degrees);
        armMotor.set(ControlMode.MotionMagic, sensorUnits);
    }

    //despite making up 13% of the population

    public void resetEncoders() {

        armMotor.getSensorCollection().setQuadraturePosition(0, 0);
    }

    @Override
    public void reset() {
        resetEncoders();
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    private void configTalon(WPI_TalonSRX talon) {
        talon.setInverted(true);
        //talon.setSensorPhase(true);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        TalonHelper.setPIDGains(talon, kArmNormalRateSlot, kArmKp, kArmKi, kArmKd, kArmKf, kArmRampRate, kArmIZone);
        TalonHelper.setMotionMagicParams(talon, kArmNormalRateSlot, kArmMaxVelocity, kArmMaxAccel);
/*
        TalonHelper.setMotionMagicParams(talon, kArmFastRateSlot, kArmMaxVelocity, kArmMaxAccelDownFast);
*/
        talon.selectProfileSlot(kArmNormalRateSlot, 0);

        armMotor.setSelectedSensorPosition(0, 0, 0);


/*
        talon.configContinuousCurrentLimit(kArmMaxContinuousCurrentLimit, 0);
        talon.configPeakCurrentLimit(kArmMaxPeakCurrentLimit, 0);
        talon.configPeakCurrentDuration(kArmMaxPeakCurrentDurationMS, 0);

        talon.enableCurrentLimit(true);

        talon.configForwardSoftLimitThreshold((int) (kArmSoftMax * kArmEncoderGearRatio * sensorUnitsPerRotationMag), kTimeoutMs);
        talon.configReverseSoftLimitThreshold((int) (kArmSoftMin * kArmEncoderGearRatio * sensorUnitsPerRotationMag), kTimeoutMs);
        talon.configForwardSoftLimitEnable(true, kTimeoutMs);
        talon.configReverseSoftLimitEnable(true, kTimeoutMs);
        talon.configAllowableClosedloopError(0, kArmAllowedError, kTimeoutMs);


        talon.selectProfileSlot(kArmNormalRateSlot, 0);

        TalonHelper.setPIDGains(talon, kArmNormalRateSlot, kArmKp, kArmKi, kArmKd, kArmKf, kArmRampRate, kArmIZone);
        TalonHelper.setPIDGains(talon, kArmFastRateSlot, kArmKp, kArmKi, kArmKd, kArmKf, kArmRampRate, kArmIZone);
        TalonHelper.setMotionMagicParams(talon, kArmNormalRateSlot, kArmMaxVelocity, kArmMaxAccel);
        TalonHelper.setMotionMagicParams(talon, kArmFastRateSlot, kArmMaxVelocity, kArmMaxAccelDownFast);*/


    }
}
