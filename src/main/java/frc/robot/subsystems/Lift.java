package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Constants;

public class Lift extends PIDSubsystem implements Section, Constants {

    private WPI_TalonSRX liftMasterMotor;
    private WPI_VictorSPX liftSlaveMotor1;
    private WPI_VictorSPX liftSlaveMotor2;
    private WPI_VictorSPX liftSlaveMotor3;

    private static Lift instance = null;

    boolean hasStopped = false;
    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;

    private Lift() {
        super(0.05, 0, 0);
        liftMasterMotor = new WPI_TalonSRX(liftMotor1ID);
        liftSlaveMotor1 = new WPI_VictorSPX(liftMotor2ID);
        liftSlaveMotor2 = new WPI_VictorSPX(liftMotor3ID);
        liftSlaveMotor3 = new WPI_VictorSPX(liftMotor4ID);
        topLimitSwitch = new DigitalInput(limitSwitchLiftBottomPort);
        bottomLimitSwitch = new DigitalInput((limitSwitchLiftTopPort));

        configTalon(liftMasterMotor, true);

    }

    public static Lift getInstance() {
        if(instance == null) {
            instance = new Lift();
        }

        return instance;
    }

    private void configTalon(WPI_TalonSRX master, boolean reversed) {
        master.set(0);

        master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        master.setSensorPhase(reversed);
        // master.configClosedloopRamp(0.25, 0);
        master.configOpenloopRamp(0.25, 0);
//      masteqr.configNominalOutputForward(NOMINAL_OUTPUT_VOLTAGE, -NOMINAL_OUTPUT_VOLTAGE);
        //    master.configPeakOutputVoltage(+PEAK_OUTPUT_VOLTAGE, -PEAK_OUTPUT_VOLTAGE);

        //aster.configSen R);

        master.configAllowableClosedloopError(0, 0, 0);

        //  master.setVoltageRampRate(48);
        master.config_kF(0, kfDriveTrainVel, 0);
        master.config_kP(0, kpDriveTrainVel, 0);
        master.config_kI(0, kiDriveTrainVel, 0);
        master.config_kD(0, kdDriveTrainVel, 0);

        master.config_kF(1, kfDriveTrainPos, 0);
        master.config_kP(1, kpDriveTrainPos, 0);
        master.config_kI(1, kiDriveTrainPos, 0);
        master.config_kD(1, kdDriveTrainPos, 0);

        master.config_kF(2, kfDriveTrainPos2, 0);
        master.config_kP(2, kpDriveTrainPos2, 0);
        master.config_kI(2, kiDriveTrainPos2, 0);
        master.config_kD(2, kdDriveTrainPos2, 0);
        //aster.setF(kfDriveTrainVbus);
        //master.setP(kpDriveTrainVbus);
        //master.setI(kiDriveTrainVbus);.config
        //     master.setD(kdDriveTrainVbus);
    }


    @Override
    protected void initDefaultCommand() {
        System.out.println("sent from my iphone");
    }

    @Override
    public void teleop(Joystick gamepad) {

        if (gamepad.getRawButton(BTN_LB)/* && bottomLimitSwitch.get()*/) {
            getPIDController().disable();
            usePIDOutput(.35);
            hasStopped = false;
        } else if (gamepad.getRawButton(BTN_RB) /*&& topLimitSwitch.get()*/ /*&& Robot.lift.potentiometer.pidGet() > Utils.iPhoneMath(.97)*/) {
            getPIDController().disable();
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

    public void stopLift() {
        if (!hasStopped) {
            getPIDController().enable();
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
            usePIDOutput(-getPIDController().get());

        //System.out.println("Er" + "ror: " + getPIDController().get() + " : " + liftMasterMotor.get() + " : " + potentiometer.pidGet());
    }

    public void autoPID(){
        getPIDController().enable();
        usePIDOutput(getPIDController().get());
    }

    @Override
    protected double returnPIDInput() {
        return potentiometer.pidGet();
    }


    @Override
    protected void usePIDOutput(double output) {
        liftMasterMotor.pidWrite(output);
        liftSlaveMotor1.pidWrite(-output);
        liftSlaveMotor2.pidWrite(-output);
        liftSlaveMotor3.pidWrite(-output);
    }
}
