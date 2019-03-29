package frc.robot.Utilities;


import frc.robot.Utilities.Constants.Constants;

public class PIDController implements Constants {
    private double i = 0, d, KP, KI,
            KD, previousError = 0, maxI,
            previousTime = 0, setpoint = 0,
            pidOut = 0;

    private boolean enabled = false;



    public PIDController(double KP, double KI, double KD, double maxI) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.maxI = maxI;
    }

    public double get(double currentLoc) {
        if (enabled) {
            double error = setpoint - currentLoc;
            double deltaTime = (System.nanoTime() - previousTime);
            i += Math.abs(currentLoc) > Math.abs(setpoint) * 0.8 ? error * deltaTime : 0;
            //i = Math.min(maxI, Math.max(-maxI, i));
            d = (error - previousError) / deltaTime;
            double power = (KP * error) + (KI * i) + (KD * d);
            previousTime = System.nanoTime();
            previousError = error;

            pidOut = power;

            return power;
        } else {
            return 0;
        }
    }

    public double getI () {
        return i;
    }

    public void reset() {
        i = 0;
        previousError = 0;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError() {
        return previousError;
    }

    public double readPIDOutput() {
        return pidOut;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setKp(double kp) {
        this.KP = kp;
    }
}