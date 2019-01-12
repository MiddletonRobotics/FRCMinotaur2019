package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;

public class Lift extends PIDSubsystem implements Section, Constants {

    WPI_TalonSRX liftMotor;
    WPI_TalonSRX liftv2Motor;
    WPI_TalonSRX liftv3Motor;
    WPI_TalonSRX liftv4Motor;

    public Potentiometer potentiometer;

    boolean hasStopped = false;
    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;

    public Lift() {
        super(0.05, 0, 0);
        liftMotor = new WPI_TalonSRX(liftMotor1Port);
        liftv2Motor = new WPI_TalonSRX(liftMotor2Port);
        liftv3Motor = new WPI_TalonSRX(liftMotor3Port);
        liftv4Motor = new WPI_TalonSRX(liftMotor4Port);
        topLimitSwitch = new DigitalInput(limitSwitchLiftBottomPort);
        bottomLimitSwitch = new DigitalInput((limitSwitchLiftTopPort));
        potentiometer = new AnalogPotentiometer(1);


        setAbsoluteTolerance(.006);
        getPIDController
                ().setContinuous(false);
        setInputRange(0, 2);
        setSetpoint(potentiometer.pidGet());
        getPIDController().enable();
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
