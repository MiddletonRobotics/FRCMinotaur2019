package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Climbaur extends Subsystem implements Section, Constants {

    VictorSP winchMotor;
    VictorSP winchv2Motor;


    boolean hasStopped = false;

    public Climbaur() {
        winchMotor = new VictorSP(winchMotor1Port);
        winchv2Motor = new VictorSP(winchMotor2Port);
        winchv2Motor.setInverted(true);
    }

    @Override
    protected void initDefaultCommand() {
        System.out.println("sent from my iphone");
    }

    @Override
    public void teleop(Joystick gamepad) {

        if (gamepad.getPOV() == DPAD_DOWN) {
            setSpeed(1);
            hasStopped = false;
        } else if (gamepad.getPOV() == DPAD_UP) {
            setSpeed(-1);
            hasStopped = false;
        } else {
            stopWinch();
        }
    }

    @Override
    public void reset() {
        hasStopped = false;
        stopWinch();

    }

    public void stopWinch() {
        if (!hasStopped) {
            hasStopped = true;
            setSpeed(0);
        }
    }



    protected void setSpeed(double speed) {
        winchMotor.set(speed);
        winchv2Motor.set(speed);

    }
}
