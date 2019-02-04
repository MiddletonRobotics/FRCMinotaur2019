package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Utilities.Drivers.TalonHelper;

public class Lift implements Section, Constants {

    private WPI_TalonSRX liftMasterMotor;
    private WPI_VictorSPX liftSlaveMotor1;
    private WPI_VictorSPX liftSlaveMotor2;
    private WPI_VictorSPX liftSlaveMotor3;

    private static Lift instance = null;

    boolean hasStopped = false;
    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;

    private Lift() {
        liftMasterMotor = new WPI_TalonSRX(liftMotor1ID);
        liftSlaveMotor1 = new WPI_VictorSPX(liftMotor2ID);
        liftSlaveMotor2 = new WPI_VictorSPX(liftMotor3ID);
        liftSlaveMotor3 = new WPI_VictorSPX(liftMotor4ID);
        topLimitSwitch = new DigitalInput(limitSwitchLiftBottomPort);
        bottomLimitSwitch = new DigitalInput((limitSwitchLiftTopPort));

        init();

    }

    public static Lift getInstance() {
        if(instance == null) {
            instance = new Lift();
        }

        return instance;
    }

    private void init() {
        liftMasterMotor.set(0);

        liftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        liftMasterMotor.setSensorPhase(true);
        // master.configClosedloopRamp(0.25, 0);
        liftMasterMotor.configOpenloopRamp(0.25, 0);
//      masteqr.configNominalOutputForward(NOMINAL_OUTPUT_VOLTAGE, -NOMINAL_OUTPUT_VOLTAGE);
        //    master.configPeakOutputVoltage(+PEAK_OUTPUT_VOLTAGE, -PEAK_OUTPUT_VOLTAGE);

        //aster.configSen R);

        liftMasterMotor.configAllowableClosedloopError(0, 0, 0);



        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP
        TalonHelper.setPIDGains(liftMasterMotor, kElevatorUpRateSlot, kElevatorKp, kElevatorKi, kElevatorKd, kElevatorKf, kElevatorRampRate, kElevatorIZone);
        TalonHelper.setPIDGains(liftMasterMotor, kElevatorDownRateSlot, kElevatorKp, kElevatorKi, kElevatorKd, kElevatorKf, kElevatorRampRate, kElevatorIZone);
        TalonHelper.setMotionMagicParams(liftMasterMotor, kElevatorUpRateSlot, kElevatorMaxVelocityUp, kElevatorMaxAccelUp);
        TalonHelper.setMotionMagicParams(liftMasterMotor, kElevatorDownRateSlot, kElevatorMaxVelocityDown, kElevatorMaxAccelDown);
        liftMasterMotor.selectProfileSlot(kElevatorUpRateSlot, 0);
        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP

        setupSlaves(liftMasterMotor, liftSlaveMotor1);
        setupSlaves(liftMasterMotor, liftSlaveMotor1);
        setupSlaves(liftMasterMotor, liftSlaveMotor1);
        setupSlaves(liftMasterMotor, liftSlaveMotor1);

    }

    private void setupSlaves(WPI_TalonSRX master, WPI_VictorSPX slave) {
        slave.follow(master);
    }

    public void setElevatorHeightInches(double height) {


        //hey dummy test limit switch pls thanks
        if (!bottomLimitSwitch.get() && height <= elevatorMinHeight) {
            zeroLift();
        }

        liftMasterMotor.set(ControlMode.MotionMagic, height * sensorUnitsPerRotationMag * kElevatorEncoderGearRatio);
    }

    public void setElevatorHeightPercent(double percent) {
        setElevatorHeightInches(elevatorHeightRange*percent/100);
    }

    protected void initDefaultCommand() {
        System.out.println("sent from my iphone");
    }

    @Override
    public void teleop(Joystick gamepad) {

        if (gamepad.getRawButton(BTN_LB)/* && bottomLimitSwitch.get()*/) {
            usePIDOutput(.35);
            hasStopped = false;
        } else if (gamepad.getRawButton(BTN_RB) /*&& topLimitSwitch.get()*/ /*&& Robot.lift.potentiometer.pidGet() > Utils.iPhoneMath(.97)*/) {
            usePIDOutput(-0.75);
            hasStopped = false;
        } else {
            stopLift();
        }
    }

    @Override
    public void reset() {
        hasStopped = false;
        stopLift();
    }

    public void zeroLift() {
        liftMasterMotor.set(ControlMode.Disabled, 0);
        int homeElevatorValue = (int)(Constants.kElevatorHome * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation);

        boolean setSucceeded;
        int retryCounter = 0;

        do {
            setSucceeded = true;

            setSucceeded &= mElevatorMotorMaster.setSelectedSensorPosition(homeElevatorValue, 0, Constants.kTimeoutMs) == ErrorCode.OK;
            setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

        } while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

        if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
            ConsoleReporter.report("Failed to zero Elevator!!!", MessageLevel.DEFCON1);

        mElevatorMotorMaster.set(ControlMode.MotionMagic, homeElevatorValue);
        double newPos = ElevatorPosition.TRUE_HOME;
        newPos += enterRehomingMode ? ElevatorPosition.TENSION_OFFSET : ElevatorPosition.TENSION_OFFSET;
        setElevatorHeight(newPos);

        if (enterRehomingMode) {
            elevatorRequestHomingTime = Timer.getFPGATimestamp();
            setElevatorRequestHoming(true);
        }

        setElevatorFaulted(false, false);

        return retryCounter < Constants.kTalonRetryCount && setSucceeded;
    }

    public void stopLift() {
        if (!hasStopped) {
            liftMasterMotor.set(0);
            liftSlaveMotor1.set(0);
            liftSlaveMotor2.set(0);
            liftSlaveMotor3.set(0);
            hasStopped = true;
        }
        /*if (!bottomLimitSwitch.get()) {
            liftMasterMotor.set(0);
            liftSlaveMotor1.set(0);
            liftSlaveMotor2.set(0);
            liftSlaveMotor3.set(0);
        } else*/

        //System.out.println("Er" + "ror: " + getPIDController().get() + " : " + liftMasterMotor.get() + " : " + potentiometer.pidGet());
    }





    protected void usePIDOutput(double output) {
        liftMasterMotor.pidWrite(output);
        liftSlaveMotor1.pidWrite(-output);
        liftSlaveMotor2.pidWrite(-output);
        liftSlaveMotor3.pidWrite(-output);
    }
}
