package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Constants.Positions.ArmPositions;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Section;

public class Arm extends PIDSubsystem implements Section, Constants {

    private WPI_TalonSRX armMotor;

    private static Arm instance = null;
    private boolean manual = true;
    private boolean armDown = false;
    private boolean aPrev = false;
    private boolean wasInCargoPickup = false;

    private Arm() {
        super(kArmKp,kArmKi,kArmKd);
        armMotor = new WPI_TalonSRX(armTalonID);
        configTalon(armMotor);
        getPIDController().setContinuous(false);
        setSetpoint(armMotor.getSensorCollection().getQuadraturePosition());
        getPIDController().enable();
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

        if (gamepad.getRawButton(BTN_A)) {
            if (!aPrev) {
                armDown = !armDown;
            }
        }

        aPrev = gamepad.getRawButton(BTN_A);

        if (gamepad.rightBumper()) {
            getPIDController().disable();
            setArmSpeedPercent(0.6);
            manual = true;
        } else if (gamepad.leftBumper()) {
            getPIDController().disable();
            setArmSpeedPercent(-0.3);
            manual = true;
        }  else if (gamepad.a() && !armDown) {
            getPIDController().enable();
/*          armMotor.setSelectedSensorPosition(ArmPositions.armDiscPosition, 0, Constants.kTimeoutMs);
            armMotor.set(ControlMode.MotionMagic, ArmPositions.armDiscPosition);*/
            setSetpoint(Robot.intake.intakeSolenoid.getValue() == DoubleSolenoid.Value.kReverse && !Robot.liftPID.getManual() ? ArmPositions.armCargoShipPosition : ArmPositions.armUpPosition);
            usePIDOutput(getPIDController().get());
            manual = false;
        } else if (gamepad.rightTriggerPressed() && Robot.intake.intakeSolenoid.getValue() == DoubleSolenoid.Value.kReverse && Robot.liftPID.getPosition() > -1000) {
            getPIDController().enable();
/*          armMotor.setSelectedSensorPosition(ArmPositions.armDiscPosition, 0, Constants.kTimeoutMs);
            armMotor.set(ControlMode.MotionMagic, ArmPositions.armDiscPosition);*/
            setSetpoint(ArmPositions.armCargoPickupPosition);
            usePIDOutput(getPIDController().get());
            wasInCargoPickup = true;
            manual = false;
        } else if (gamepad.a() && armDown || wasInCargoPickup) {
            getPIDController().enable();
/*          armMotor.setSelectedSensorPosition(ArmPositions.armDiscPosition, 0, Constants.kTimeoutMs);
            armMotor.set(ControlMode.MotionMagic, ArmPositions.armDiscPosition);*/
            setSetpoint(ArmPositions.armDiscPosition);
            usePIDOutput(getPIDController().get());
            manual = false;
            wasInCargoPickup = false;
        } else {
            stop();
        }

/*
        System.out.println(armMotor.getSensorCollection().getQuadraturePosition());
*/

        //System.out.println(/*armMotor.getSensorCollection().getQuadraturePosition()*/armMotor.getMotorOutputPercent());
/*        if (gamepad.rightBumper()) {
            armMotor.set(0.6);
        } else if (gamepad.leftBumper()) {
            armMotor.set(-0.3);
        } else {
            armMotor.set(0.055);
        }*/
    }

    private void stop() {
        if (manual) {
            setSetpoint(armMotor.getSensorCollection().getQuadraturePosition());
            manual = false;
        }
/*
        getPIDController().setF(Math.cos(getRotationAngle())*kArmKf);
*/
        getPIDController().enable();
        usePIDOutput(getPIDController().get());
/*
        System.out.println("Forward: " + armMotor.getSensorCollection().isFwdLimitSwitchClosed());
        System.out.println("Reverse: " + armMotor.getSensorCollection().isRevLimitSwitchClosed());*/
    }

/*    private void zeroArm() {
        armMotor.set(ControlMode.Disabled, 0);

        armMotor.setSelectedSensorPosition(ArmPositions.armHomePosition, 0, Constants.kTimeoutMs);
        armMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
        armMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        armMotor.set(ControlMode.MotionMagic, ArmPositions.armHomePosition);
    }*/

    private void zeroArm() {

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
        /*armMotor.getSensorCollection().setPulseWidthPosition(0, 0);*/
    }

    public void setArmSpeedPercent(double speed) {
        /*speed = speed < 0 && getLimitPressed() ? 0 : speed;*/
        armMotor.set(ControlMode.PercentOutput, speed);

    }

    @Override
    public void reset() {
        resetEncoders();
        setArmSpeedPercent(0);
    }

    private void configTalon(WPI_TalonSRX talon) {
        talon.setInverted(true);
        //talon.setSensorPhase(true);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

        TalonHelper.setPIDGains(talon, kArmNormalRateSlot, kArmKp, kArmKi, kArmKd, kArmKf, kArmRampRate, kArmIZone);
        TalonHelper.setMotionMagicParams(talon, kArmNormalRateSlot, kArmMaxVelocity, kArmMaxAccel);
/*
        TalonHelper.setMotionMagicParams(talon, kArmFastRateSlot, kArmMaxVelocity, kArmMaxAccelDownFast);
*/
        talon.selectProfileSlot(kArmNormalRateSlot, 0);

        armMotor.setSelectedSensorPosition(0, 0, 0);

        armMotor.setControlFramePeriod(10, kTimeoutMs);


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

    public double getRotationAngle() {
        return armDegreeOffset*(Math.PI/180) - 2*Math.PI*armMotor.getSensorCollection().getQuadraturePosition()/(2*sensorUnitsPerRotationMag);
    }

    @Override
    protected double returnPIDInput() {
        return armMotor.getSensorCollection().getQuadraturePosition();
    }

    @Override
    protected void usePIDOutput(double output) {
        armMotor.pidWrite(-output);
    }

    public boolean getLimitPressed() {
        return !armMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }
}
