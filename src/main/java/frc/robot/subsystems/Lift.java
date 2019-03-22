package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Constants.Positions.LiftPositions;
import frc.robot.Utilities.Section;
import frc.robot.Utilities.Utils;

public class Lift implements Section, Constants {

    private WPI_TalonSRX liftMasterMotor;
    private WPI_VictorSPX liftSlaveMotor1;
    private WPI_VictorSPX liftSlaveMotor2;
    private WPI_VictorSPX liftSlaveMotor3;

    private static Lift instance = null;

/*    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;*/

    private Lift() {
        liftMasterMotor = new WPI_TalonSRX(liftMasterID);
        liftSlaveMotor1 = new WPI_VictorSPX(liftSlave1ID);
        liftSlaveMotor2 = new WPI_VictorSPX(liftSlave2ID);
        liftSlaveMotor3 = new WPI_VictorSPX(liftSlave3ID);
        /*topLimitSwitch = new DigitalInput(limitSwitchLiftBottomPort);
        bottomLimitSwitch = new DigitalInput(limitSwitchLiftTopPort);*/

        configTalon(liftMasterMotor);

    }

    public static Lift getInstance() {
        if(instance == null) {
            instance = new Lift();
        }

        return instance;
    }

    private void configTalon(WPI_TalonSRX motor) {
        motor.set(0);

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
        motor.setSensorPhase(true);
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

//        System.out.println(liftMasterMotor.getSensorCollection().getQuadraturePosition());

        if (gamepad.y()/* && isLiftHeigherThanMax()*//* && bottomLimitSwitch.get()*/) { //hey dummy test limit switch pls thanks
            /*setLiftSpeedPercent(0.1);*/
            //liftSlaveMotor1.set(0.1);      //UP
/*            liftSlaveMotor2.set(0.1);
            liftSlaveMotor3.set(0.1);*/
            setLiftSpeedPercent(-0.6);
        } else if (gamepad.b() /*&& isLiftLowerThanMin()*/ /*&& topLimitSwitch.get()*/ /*&& Robot.lift.potentiometer.pidGet() > Utils.iPhoneMath(.97)*/) {
            /*setLiftSpeedPercent(-0.1);*/
            //liftSlaveMotor1.set(-0.1);           //DOWN
/*            liftSlaveMotor2.set(-0.1);
            liftSlaveMotor3.set(-0.1);*/
            setLiftSpeedPercent(0.6);
        } else {
            setLiftSpeedPercent(-0.02);
        }
    }

    @Override
    public void reset() {
        /*zeroLift();*/
        stopLift();
//        resetEnoder();
    }

    public void zeroLift() {

        liftMasterMotor.set(ControlMode.Disabled, 0);
        int homeLiftValue = (int)(LiftPositions.liftHomePosition * kLiftEncoderGearRatio * sensorUnitsPerRotationMag);

        liftMasterMotor.setSelectedSensorPosition(homeLiftValue, 0, Constants.kTimeoutMs);
        liftMasterMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        liftMasterMotor.set(ControlMode.MotionMagic, homeLiftValue);

    }

    public void resetEnoder() {
        liftMasterMotor.set(ControlMode.PercentOutput, -0.1);
        liftSlaveMotor1.set(ControlMode.PercentOutput, -0.1);
        liftSlaveMotor2.set(ControlMode.PercentOutput, -0.1);
        liftSlaveMotor3.set(ControlMode.PercentOutput, -0.1);
        while (liftMasterMotor.getOutputCurrent() < 10) {
            Utils.sleep(1);
        }
        liftMasterMotor.set(ControlMode.PercentOutput, 0);
        liftSlaveMotor1.set(ControlMode.PercentOutput, 0);
        liftSlaveMotor2.set(ControlMode.PercentOutput, 0);
        liftSlaveMotor3.set(ControlMode.PercentOutput, 0);

        liftMasterMotor.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
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
        liftMasterMotor.set(ControlMode.PercentOutput, speed);
        liftSlaveMotor1.set(ControlMode.PercentOutput, speed);
        liftSlaveMotor2.set(ControlMode.PercentOutput, speed);
        liftSlaveMotor3.set(ControlMode.PercentOutput, speed);
    }

    public void setLiftSpeed(double speed) {
        liftMasterMotor.set(ControlMode.Velocity, speed);
    }

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
}
