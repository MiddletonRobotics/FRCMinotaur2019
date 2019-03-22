package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Constants.Positions.ArmPositions;
import frc.robot.Utilities.Constants.Positions.LiftPositions;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Section;
import frc.robot.Utilities.Utils;

public class LiftPID extends PIDSubsystem implements Section, Constants {

    private WPI_TalonSRX liftMasterMotor;
    private WPI_VictorSPX liftSlaveMotor1;
    private WPI_VictorSPX liftSlaveMotor2;
    private WPI_VictorSPX liftSlaveMotor3;
    private DigitalInput limitSwitch;

    private static LiftPID instance = null;
    private int position = 0;
    private boolean manual = true;

/*    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;*/

    private LiftPID() {
        super(kLiftKp, kLiftKi, kLiftKd);
        liftMasterMotor = new WPI_TalonSRX(liftMasterID);
        liftSlaveMotor1 = new WPI_VictorSPX(liftSlave1ID);
        liftSlaveMotor2 = new WPI_VictorSPX(liftSlave2ID);
        liftSlaveMotor3 = new WPI_VictorSPX(liftSlave3ID);
        limitSwitch = new DigitalInput(limitSwitchPort);
        /*topLimitSwitch = new DigitalInput(limitSwitchLiftBottomPort);
        bottomLimitSwitch = new DgiitalInput(limitSwitchLiftTopPort);*/

        configTalon(liftMasterMotor);

        getPIDController().setContinuous(false);
        setSetpoint(liftMasterMotor.getSensorCollection().getQuadraturePosition());
        getPIDController().enable();
    }

    public static LiftPID getInstance() {
        if(instance == null) {
            instance = new LiftPID();
        }

        return instance;
    }

    public double getPosition() {
        return liftMasterMotor.getSensorCollection().getQuadraturePosition();
    }

    private void configTalon(WPI_TalonSRX motor) {
        motor.set(0);

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
        motor.setSensorPhase(true);
        motor.setInverted(true);
        // master.configClosedloopRamp(0.25, 0);
/*
        motor.configOpenloopRamp(2, kTimeoutMs);
*/
//      masteqr.configNominalOutputForward(NOMINAL_OUTPUT_VOLTAGE, -NOMINAL_OUTPUT_VOLTAGE);
        //    master.configPeakOutputVoltage(+PEAK_OUTPUT_VOLTAGE, -PEAK_OUTPUT_VOLTAGE);

        //aster.configSen R);

        motor.configAllowableClosedloopError(0, 0, kTimeoutMs);



        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP
        TalonHelper.setPIDGains(motor, kLiftUpRateSlot, kLiftKp, kLiftKi, kLiftKd, kLiftKf, kLiftRampRate, kLiftIZone);
        TalonHelper.setPIDGains(motor, kLiftDownRateSlot, kLiftKp, kLiftKi, kLiftKd, kLiftKf, kLiftRampRate, kLiftIZone);
        TalonHelper.setMotionMagicParams(motor, kLiftUpRateSlot, kLiftMaxVelocityUp, kLiftMaxAccelUp);
        TalonHelper.setMotionMagicParams(motor, kLiftDownRateSlot, kLiftMaxVelocityDown, kLiftMaxAccelDown);
        motor.selectProfileSlot(kLiftUpRateSlot, 0);
        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP

        setupSlaves(motor, liftSlaveMotor1);
        setupSlaves(motor, liftSlaveMotor2);
        setupSlaves(motor, liftSlaveMotor3);

        motor.configForwardSoftLimitEnable(false, kTimeoutMs);
        motor.configReverseSoftLimitEnable(false, kTimeoutMs);

        motor.setNeutralMode(NeutralMode.Brake);
        liftSlaveMotor1.setNeutralMode(NeutralMode.Brake);
        liftSlaveMotor2.setNeutralMode(NeutralMode.Brake);
        liftSlaveMotor3.setNeutralMode(NeutralMode.Brake);


    }

    private void setupSlaves(WPI_TalonSRX master, WPI_VictorSPX slave) {
        slave.follow(master);
    }



    public void initDefaultCommand() {
        System.out.println("sent from my iphone");
    }


    @Override
    public void teleop(MinoGamepad gamepad) {

/*
        System.out.println(liftMasterMotor.getSensorCollection().getQuadraturePosition());
*/
        position += gamepad.getRawButtonReleased(DPAD_NORTH) ? position < 3 ? 1 : 0 : gamepad.getRawButtonReleased(DPAD_SOUTH) ? position > 0 ? -1 : 0 : 0;

        if (gamepad.y()) {
            getPIDController().disable();
            setLiftSpeedPercent(0.6);
            manual = true;
        } else if (gamepad.b()) {
            getPIDController().disable();
            setLiftSpeedPercent(-0.6);
            manual = true;
        } /*else if (gamepad.dpadRight()) {
            getPIDController().enable();
            setSetpoint((LiftPositions.liftHomePosition));

            usePIDOutput((getPIDController().get()));
        }*/ else if (gamepad.dpadLeft()) {
            getPIDController().enable();
            setSetpoint(LiftPositions.liftHomePosition);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else if (gamepad.dpadDown()) {
            getPIDController().enable();
            setSetpoint(Robot.intake.intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ?  LiftPositions.liftFirstHeight : LiftPositions.liftRocketFirstHeight);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else if (gamepad.dpadRight()) {
            getPIDController().enable();
            setSetpoint(Robot.intake.intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ?  LiftPositions.liftSecondHeight : LiftPositions.liftRocketSecondHeight);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else if (gamepad.dpadUp()) {
            getPIDController().enable();
            setSetpoint(Robot.intake.intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ?  LiftPositions.liftThirdHeight : LiftPositions.liftBallCargoHeight);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else {
            stopPID();
//            setLiftSpeedPercent(0);
        }


        System.out.println(getSetpoint());

/*
        System.out.println("Input: " + gamepad.);
*/

/*        if (gamepad.y()) {
            getPIDController().disable();
            setLiftSpeedPercent(-0.6);
            manual = true;
        } else if (gamepad.b()) {
            getPIDController().disable();
            setLiftSpeedPercent(0.6);
            manual = true;
        } *//*else if (gamepad.dpadRight()) {
            getPIDController().enable();
            setSetpoint((LiftPositions.liftHomePosition));

            usePIDOutput((getPIDController().get()));
        }*//* else if (position == 0 && gamepad.getRawButtonReleased(DPAD_NORTH) || gamepad.getRawButtonReleased(DPAD_SOUTH)) {
            getPIDController().enable();
            setSetpoint(LiftPositions.liftHomePosition);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else if (position == 1 && gamepad.getRawButtonReleased(DPAD_NORTH) || gamepad.getRawButtonReleased(DPAD_SOUTH)) {
            getPIDController().enable();
            setSetpoint(LiftPositions.liftFirstHeight);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else if (position == 2 && gamepad.getRawButtonReleased(DPAD_NORTH) || gamepad.getRawButtonReleased(DPAD_SOUTH)) {
            getPIDController().enable();
            setSetpoint(LiftPositions.liftSecondHeight);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else if (position == 3 && gamepad.getRawButtonReleased(DPAD_NORTH) || gamepad.getRawButtonReleased(DPAD_SOUTH)) {
            getPIDController().enable();
            setSetpoint(LiftPositions.liftThirdHeight);
            usePIDOutput((getPIDController().get()));
            manual = false;
        } else {
            stopPID();
//            setLiftSpeedPercent(0);
        }*/
    }

    private void stopPID() {
        if (manual) {
            setSetpoint(liftMasterMotor.getSensorCollection().getQuadraturePosition());
        }
        getPIDController().setF(kLiftKf);
        getPIDController().enable();
        usePIDOutput(getPIDController().get());

    }
    @Override
    public void reset() {
        zeroLift();
    }

/*    public void zeroLift() {

        liftMasterMotor.set(ControlMode.Disabled, 0);
        int homeLiftValue = (int)(LiftPositions.liftHomePosition * kLiftEncoderGearRatio * sensorUnitsPerRotationMag);

        liftMasterMotor.setSelectedSensorPosition(homeLiftValue, 0, Constants.kTimeoutMs);
        liftMasterMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        liftMasterMotor.set(ControlMode.MotionMagic, homeLiftValue);

    }*/

    public void resetEnoder() {
        liftMasterMotor.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    }

    public void zeroLift() {
        new Thread( () -> {
            long startTime = System.currentTimeMillis();
            while (limitSwitch.get() && System.currentTimeMillis() - startTime < 3000) {
                setLiftSpeedPercent(-0.2);
            }
            stopLift();
            resetEnoder();
        }).start();
    }


    public void stopLift() {
            liftMasterMotor.set(0);
            liftSlaveMotor1.set(0);
            liftSlaveMotor2.set(0);
            liftSlaveMotor3.set(0);

        /*if (!bottomLimitSwitch.get()) {
            liftMasterMotor.set(0);
            liftSlaveMotor1.set(0);
            liftSlaveMotor2.set(0);
            liftSlaveMotor3.set(0);
        } else*/

        //System.out.println("Er" + "ror: " + getPIDController().get() + " : " + liftMasterMotor.get() + " : " + potentiometer.pidGet());
    }

    public void setLiftHeightInches(double heightInches) {
        //hey dummy test limit switch pls thanks
/*        if (!bottomLimitSwitch.get() && height <= LiftPositions.liftMinHeight) {
            zeroLift();
        }*/

        double heightSensorUnits = inchesToSensorUnits(heightInches);
        setLiftHeightSensorUnits(clipHeightSensorUnits(heightSensorUnits));
    }

    public void setLiftSpeedPercent(double speed) {
        speed = speed < 0 && !limitSwitch.get() ? 0 : (speed < -.3 ? (getPosition() > LiftPositions.liftSlowHeight? -0.3: speed) : speed);
/*
        speed = getPosition() < LiftPositions.liftMaxHeight ? (speed > 0 ? 0: speed) : speed;
*/
        liftMasterMotor.set(ControlMode.PercentOutput, speed);
        liftSlaveMotor1.set(ControlMode.PercentOutput, speed);
        liftSlaveMotor2.set(ControlMode.PercentOutput, speed);
        liftSlaveMotor3.set(ControlMode.PercentOutput, speed);
    }

/*    public void setLiftSpeed(double speed) {
        liftMasterMotor.set(ControlMode.Velocity, speed);
    }*/

    public void setLiftHeightSensorUnits(double sensorUnits) {
        //hey dummy test limit switch pls thanks
/*        if (!bottomLimitSwitch.get() && height <= LiftPositions.liftMinHeight) {
            zeroLift();
        }*/

        liftMasterMotor.set(ControlMode.MotionMagic, sensorUnits);
    }

    public double inchesToSensorUnits(double inches) {
        return ((inches - liftHeightToGround)/(Math.PI * liftSprocketDiameter))*sensorUnitsPerRotationMag;
    }

    public double getHeightSensorUnitsFromPercent (double percent) {
        return LiftPositions.liftHeightRange*percent + LiftPositions.liftMinHeight;
    }

    public boolean isGreaterThanMaxSensorUnits(double height) {
        return height > LiftPositions.liftMaxHeight;
    }

    public boolean isLessThanMinSensorUnits(double height) {
        return height < LiftPositions.liftMinHeight;
    }

    public boolean isLiftLowerThanMin() {
       return liftMasterMotor.getSensorCollection().getQuadraturePosition() < LiftPositions.liftMinHeight;
    }

    public boolean isLiftHeigherThanMax() {
        return liftMasterMotor.getSensorCollection().getQuadraturePosition() > LiftPositions.liftMaxHeight;
    }

    public double clipHeightSensorUnits(double heightSensorUnits) {
        return Math.min(LiftPositions.liftMaxHeight, Math.max(LiftPositions.liftMinHeight, heightSensorUnits));
    }

    @Override
    protected double returnPIDInput() {
        return liftMasterMotor.getSensorCollection().getQuadraturePosition();
    }

    @Override
    protected void usePIDOutput(double output) {
        setLiftSpeedPercent(-output);
    }
}
