package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Positions.Elevator;
import frc.robot.Utilities.Section;

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
        TalonHelper.setPIDGains(motor, kElevatorUpRateSlot, kElevatorKp, kElevatorKi, kElevatorKd, kElevatorKf, kElevatorRampRate, kElevatorIZone);
        TalonHelper.setPIDGains(motor, kElevatorDownRateSlot, kElevatorKp, kElevatorKi, kElevatorKd, kElevatorKf, kElevatorRampRate, kElevatorIZone);
        TalonHelper.setMotionMagicParams(motor, kElevatorUpRateSlot, kElevatorMaxVelocityUp, kElevatorMaxAccelUp);
        TalonHelper.setMotionMagicParams(motor, kElevatorDownRateSlot, kElevatorMaxVelocityDown, kElevatorMaxAccelDown);
        motor.selectProfileSlot(kElevatorUpRateSlot, 0);
        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP

        setupSlaves(motor, liftSlaveMotor1);
        setupSlaves(motor, liftSlaveMotor1);
        setupSlaves(motor, liftSlaveMotor1);
        setupSlaves(motor, liftSlaveMotor1);

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
        int homeElevatorValue = (int)(Elevator.elevatorHomePosition * kElevatorEncoderGearRatio * sensorUnitsPerRotationMag);

        liftMasterMotor.setSelectedSensorPosition(homeElevatorValue, 0, Constants.kTimeoutMs);
        liftMasterMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        liftMasterMotor.set(ControlMode.MotionMagic, homeElevatorValue);

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
