package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;

public class IntakeTilt extends PIDSubsystem implements Constants, Section {

    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    private Potentiometer potentiometer;

    private VictorSP tiltMotor;

    private boolean isToggle;
    private boolean hasStopped = false;
    private boolean isMan = false;

    public IntakeTilt() {
        super(10, 0, 0.0000);

        topLimitSwitch = new DigitalInput(limitSwitchTiltTopPort);
        bottomLimitSwitch = new DigitalInput(limitSwitchTiltBottomPort);
        setAbsoluteTolerance(.003);
        getPIDController
                ().setContinuous(false);
        setInputRange(0, 1.5);

        tiltMotor = new VictorSP(tiltMotorPort);

        potentiometer = new AnalogPotentiometer(intakeTiltPotPort);

        setSetpoint(potentiometer.pidGet());
        getPIDController().enable();

    }

    @Override
    protected void initDefaultCommand() {
        System.out.println("IntakeTilt Jeff");
    }

    @Override
    public void teleop(Joystick gamepad) {
        if (gamepad.getRawButton(BTN_START) && !isToggle) {
            isToggle = true;
            isMan = !isMan;
        }
        if (!gamepad.getRawButton(BTN_START))
            isToggle = false;
        if (isMan) {
            if (gamepad.getRawButton(BTN_X) /*&& topLimitSwitch.get()*/) {
                getPIDController().disable();
                tiltMotor.set(-.75);
                hasStopped = false;
            } else if (gamepad.getRawButton(BTN_A) /*&& bottomLimitSwitch.get()*/) {
                getPIDController().disable();
                tiltMotor.set(.5);
                hasStopped = false;
            } else {
                stop();
            }
        } else {
            if (gamepad.getRawButton(BTN_Y) /*&& topLimitSwitch.get()*/) {
                setSetpoint(TILT_TOP); //Top of the Intake
                usePIDOutput(getPIDController().get());
                Robot.intake.setSpeedLimit(0.5);

                hasStopped = false;
            } else if (gamepad.getRawButton(BTN_X)) {

                setSetpoint(TILT_MIDDLE); //Middle of the intake
                usePIDOutput(getPIDController().get());
                Robot.intake.setSpeedLimit(0.5);

                hasStopped = false;
            } else if (gamepad.getRawButton(BTN_A) /*&& bottomLimitSwitch.get()*/) {
                //if (Robot.lift.potentiometer.pidGet() < Utils.iPhoneMath(.0377)) {  //This makes the lift go up if it is too low when deploying the intake
                    setSetpoint(TILT_BOTTOM); //Bottom Position
                    usePIDOutput(getPIDController().get());
                    Robot.intake.setSpeedLimit(0.5);

                    hasStopped = false;

                //} else
                    //Robot.lift.setSetpoint(Utils.iPhoneMath(.06));
            } /*else if (gamepad.getPOV() == DPAD_RIGHT && Robot.lift.potentiometer.pidGet() < Utils.iPhoneMath(.8)) {
                setSetpoint(TILT_BACK); //Bottom Position
                usePIDOutput(getPIDController().get());
                Robot.intake.setSpeedLimit(1);


                hasStopped = false;
            } */else {
                stop();
            }
        }

    }

    public void stop() {
        if (!hasStopped) {
//            getPIDController().setPID(20, 0.0000000, 0.0000);
            tiltMotor.set(0);
            if (isMan) {
                this.setSetpoint(potentiometer.pidGet());
//                getPIDController().enable();
            }
            getPIDController().enable();
            hasStopped = true;
        }
        getPIDController().setF(Math.cos((potentiometer.pidGet() - TILT_BOTTOM) / 0.0027777777) * .155);

        usePIDOutput(getPIDController().get());
//        tiltMotor.set(0);
        //System.out.println("Error: " + getPIDController().getError() + " : " + tiltMotor.get() + " : " + potentiometer.pidGet());

    }

    @Override
    public void reset() {
        tiltMotor.set(0);
        hasStopped = false;
    }

    @Override
    protected double returnPIDInput() {
        return potentiometer.pidGet();
    }


    @Override
    protected void usePIDOutput(double output) {
     tiltMotor.pidWrite(-output);
    }
}

