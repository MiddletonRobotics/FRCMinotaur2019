package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Constants;

public class Lift extends PIDSubsystem implements Section, Constants {

    private WPI_TalonSRX liftMotor;
    private WPI_TalonSRX liftv2Motor;
    private WPI_TalonSRX liftv3Motor;
    private WPI_TalonSRX liftv4Motor;
    private Potentiometer potentiometer;

    private static Lift instance = null;

    boolean hasStopped = false;
    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;

    private Lift() {
        super(0.05, 0, 0);
        liftMotor = new WPI_TalonSRX(liftMotor1ID);
        liftv2Motor = new WPI_TalonSRX(liftMotor2ID);
        liftv3Motor = new WPI_TalonSRX(liftMotor3ID);
        liftv4Motor = new WPI_TalonSRX(liftMotor4ID);
        topLimitSwitch = new DigitalInput(limitSwitchLiftBottomPort);
        bottomLimitSwitch = new DigitalInput((limitSwitchLiftTopPort));
        potentiometer = new AnalogPotentiometer(1);

    }

    public static Lift getInstance() {
        if(instance == null) {
            instance = new Lift();
        }

        return instance;
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
            setSetpoint(potentiometer.pidGet());
            liftMotor.set(0);
            liftv2Motor.set(0);
            liftv3Motor.set(0);
            liftv4Motor.set(0);
            hasStopped = true;
        }
        /*if (!bottomLimitSwitch.get()) {
            liftMotor.set(0);
            liftv2Motor.set(0);
            liftv3Motor.set(0);
            liftv4Motor.set(0);
        } else*/
            usePIDOutput(-getPIDController().get());

        //System.out.println("Er" + "ror: " + getPIDController().get() + " : " + liftMotor.get() + " : " + potentiometer.pidGet());
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
        liftMotor.pidWrite(output);
        liftv2Motor.pidWrite(-output);
        liftv3Motor.pidWrite(-output);
        liftv4Motor.pidWrite(-output);
    }
}
