package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Constants.Positions.LiftPositions;
import frc.robot.Utilities.Section;

public class Lift implements Section, Constants {

    private WPI_TalonSRX liftMasterMotor;
    private WPI_VictorSPX liftSlaveMotor1;
    private WPI_VictorSPX liftSlaveMotor2;
    private WPI_VictorSPX liftSlaveMotor3;

    private static Lift instance = null;

    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;

    private Lift() {
        liftMasterMotor = new WPI_TalonSRX(liftMotor1ID);
        liftSlaveMotor1 = new WPI_VictorSPX(liftMotor2ID);
        liftSlaveMotor2 = new WPI_VictorSPX(liftMotor3ID);
        liftSlaveMotor3 = new WPI_VictorSPX(liftMotor4ID);
        topLimitSwitch = new DigitalInput(limitSwitchLiftBottomPort);
        bottomLimitSwitch = new DigitalInput(limitSwitchLiftTopPort);

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

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        motor.setSensorPhase(true);
        // master.configClosedloopRamp(0.25, 0);
        motor.configOpenloopRamp(0.25, 0);
//      masteqr.configNominalOutputForward(NOMINAL_OUTPUT_VOLTAGE, -NOMINAL_OUTPUT_VOLTAGE);
        //    master.configPeakOutputVoltage(+PEAK_OUTPUT_VOLTAGE, -PEAK_OUTPUT_VOLTAGE);

        //aster.configSen R);

        motor.configAllowableClosedloopError(0, 0, 0);



        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP
        TalonHelper.setPIDGains(motor, kLiftUpRateSlot, kLiftKp, kLiftKi, kLiftKd, kLiftKf, kLiftRampRate, kLiftIZone);
        TalonHelper.setPIDGains(motor, kLiftDownRateSlot, kLiftKp, kLiftKi, kLiftKd, kLiftKf, kLiftRampRate, kLiftIZone);
        TalonHelper.setMotionMagicParams(motor, kLiftUpRateSlot, kLiftMaxVelocityUp, kLiftMaxAccelUp);
        TalonHelper.setMotionMagicParams(motor, kLiftDownRateSlot, kLiftMaxVelocityDown, kLiftMaxAccelDown);
        motor.selectProfileSlot(kLiftUpRateSlot, 0);
        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP

        setupSlaves(motor, liftSlaveMotor1);
        setupSlaves(motor, liftSlaveMotor1);
        setupSlaves(motor, liftSlaveMotor1);
        setupSlaves(motor, liftSlaveMotor1);

    }

    private void setupSlaves(WPI_TalonSRX master, WPI_VictorSPX slave) {
        slave.follow(master);
    }

    public void setLiftHeightRotations(double height) {


        //hey dummy test limit switch pls thanks
/*        if (!bottomLimitSwitch.get() && height <= LiftPositions.liftMinHeight) {
            zeroLift();
        }*/

        height = height < LiftPositions.liftMinHeight ? LiftPositions.liftMinHeight : height > LiftPositions.liftMaxHeight ? LiftPositions.liftMaxHeight : height;
        liftMasterMotor.set(ControlMode.MotionMagic, getSensorPositionFromHeight(height));
    }

    public void setLiftHeightPercent(double percent) {
        setLiftHeightRotations(LiftPositions.liftHeightRange*percent/100);
    }

    public void initDefaultCommand() {
        System.out.println("sent from my iphone");
    }

    public void setLiftSpeedPercent(double speed) {
        liftMasterMotor.set(speed);
    }

    public void setLiftSpeedSpeed(double speed) {
        liftMasterMotor.set(ControlMode.Velocity, speed);
    }


    @Override
    public void teleop(Joystick gamepad) {

        if (gamepad.getRawButton(BTN_LB) && liftMasterMotor.getSensorCollection().getQuadraturePosition() > getSensorPositionFromHeight(LiftPositions.liftMinHeight) /* && bottomLimitSwitch.get()*/) { //hey dummy test limit switch pls thanks
            setLiftSpeedPercent(0.5);
        } else if (gamepad.getRawButton(BTN_RB) && liftMasterMotor.getSensorCollection().getQuadraturePosition() < getSensorPositionFromHeight(LiftPositions.liftMaxHeight) /*&& topLimitSwitch.get()*/ /*&& Robot.lift.potentiometer.pidGet() > Utils.iPhoneMath(.97)*/) {
            setLiftSpeedPercent(-0.5);
        } else {
            stopLift();
        }
    }

    @Override
    public void reset() {
        zeroLift();
        stopLift();
    }

    public void zeroLift() {

        liftMasterMotor.set(ControlMode.Disabled, 0);
        int homeLiftValue = (int)(LiftPositions.liftHomePosition * kLiftEncoderGearRatio * sensorUnitsPerRotationMag);

        liftMasterMotor.setSelectedSensorPosition(homeLiftValue, 0, Constants.kTimeoutMs);
        liftMasterMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        liftMasterMotor.set(ControlMode.MotionMagic, homeLiftValue);

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

    public double getSensorPositionFromHeight(double height) {
        return height * sensorUnitsPerRotationMag * kLiftEncoderGearRatio;
    }

}
