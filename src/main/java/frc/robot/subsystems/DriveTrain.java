package frc.robot.subsystems;

import java.util.Set;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoInterruptedException;
import frc.robot.Constants;
import frc.robot.Robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem implements Constants, Section {

    private final WPI_TalonSRX leftTalon, rightTalon;
    private final WPI_TalonSRX leftSlave, rightSlave;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private AHRS getGyro() {
        return gyro;
    }

    public double getGyroAngle() {
        return getGyro().getAngle();
    }

    public WPI_TalonSRX getLeftTalon() {
        return this.leftTalon;
    }

    public WPI_TalonSRX getRightTalon() {
        return this.rightTalon;
    }

    public enum Direction {
        FORWARD(+1.0), BACKWARD(-1.0), CLOCKWISE(+1.0), COUNTERCLOCKWISE(-1.0);
        public final double value;

        Direction(double value) {
            this.value = value;
        }
    }

    public enum TeleopDriveModes {
        TANK_DRIVE, NEED_4_SPEED
    }

    private TeleopDriveModes driveMode = TeleopDriveModes.NEED_4_SPEED;
    private int clicksRemaining;

    public DriveTrain() {
        leftTalon = new WPI_TalonSRX(masterLeftPort);
        leftSlave = new WPI_TalonSRX(slaveLeftPort);

        rightTalon = new WPI_TalonSRX(masterRightPort);
        rightSlave = new WPI_TalonSRX(slaveRightPort);

        setupSlaves(leftTalon, leftSlave);
        setupSlaves(rightTalon, rightSlave);

        configTalon(leftTalon, true);
        configTalon(rightTalon, true);
    }

    @Override
    public void reset() {
        leftTalon.set(0);
        rightTalon.set(0);
        resetEncoders();
        resetGyro();
        setProfile(0);

    }

    private void setupSlaves(WPI_TalonSRX master, WPI_TalonSRX slave) {
        slave.set(ControlMode.Follower, master.getDeviceID());
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

    public void setLeftTarget(double target, ControlMode controlMode) {
        leftTalon.set(controlMode, target);
    }

    public void setRightTarget(double target, ControlMode controlMode) {
        rightTalon.set(controlMode, target);
    }

    public void setTarget(double left, double right, ControlMode controlMode) {
        setLeftTarget(left, controlMode);
        setRightTarget(right, controlMode);
    }

    public void setTarget(double target, ControlMode controlMode) {
        setTarget(target, target, controlMode);
    }

    public void stopDrive() {
        setTarget(0, ControlMode.PercentOutput);
    }

    public void resetEncoders() {
        stopDrive();
        leftTalon.getSensorCollection().setQuadraturePosition(0, 10);
        rightTalon.getSensorCollection().setQuadraturePosition(0, 10);

    }

    public void resetGyro() {
        gyro.reset();

    }

    public void driveByGyroOnly() {

    }

    public void driveInMode(ControlMode mode, double left, double right) {
        setTarget(left, right, mode);
    }

    public void driveInMode(ControlMode mode, double target) {
        driveInMode(mode, target, target);
    }

    public void drivePosition(double left, double right) {
        driveInMode(ControlMode.Position, left, right);
    }

    public void drivePosition(double target) {
        drivePosition(target, target);
    }

    public void driveVbus(double left, double right) {
        driveInMode(ControlMode.PercentOutput, left, right);
    }

    public void driveVbus(double target) {
        driveVbus(target, target);
    }

    public void driveVelocity(double left, double right) {

        if (left == 0 && right == 0) {
            leftTalon.set(0);
            rightTalon.set(0);
        } else {
            setTarget(left, right, ControlMode.Velocity);
        }
        // driveInMode(ControlMode.Speed, left, right);
    }

    public void driveVelocity(double target) {
        driveVelocity(target, target);
    }

    public void driveDistance(double distInches) {
        resetEncoders();
        double clicks = distInches * CLICKS_PER_INCH;

        this.driveInMode(ControlMode.Position, -clicks, clicks);
//        System.out.println(clicks + " : " + getLeftTalon().getClosedLoopError(0));
        // driveInMode(ControlMode.Position, distInches * CLICKS_PER_INCH);
    }

    public double getLeftVelocity() {
        return leftTalon.getSensorCollection().getQuadratureVelocity();
    }

    private double clip(double value, double min, double max) {
        if (value < min)
            value = min;
        if (value > max)
            value = max;
        return value;
    }

    public void setProfile(int slotID) {
        leftTalon.selectProfileSlot(slotID, 0);
        rightTalon.selectProfileSlot(slotID, 0);
    }

    @Deprecated
    public void moveByDistance(double inches, Direction direction, double speed) {
        resetEncoders();

        int targetClicks = (int) (inches * CLICKS_PER_INCH);
        int clicksRemaining;
        double inchesRemaining;
        double power;
        double p_gain = 0.3;

        do {
            clicksRemaining = targetClicks - Math.abs(rightTalon.getSensorCollection().getQuadraturePosition());
            inchesRemaining = clicksRemaining / CLICKS_PER_INCH;
            power = direction.value * speed * inchesRemaining * p_gain;
            setTarget(power, ControlMode.Position);
            if (Robot.isTeleop)
                break;
        } while (inchesRemaining > 0.5);
        stopDrive();
    }

    public double inchesToApporvas(double inches) {
        return inches / (4 * Math.PI);
    }

    public void turnP(double degrees, Direction direction, double speed, double allowableError, double timeKill, boolean xd) throws AutoInterruptedException {
        resetGyro();
        double error;
        double power;
        double kp = 0.04;
        double integral = 0;
        double ki = 0.00005;
        double prevTime = System.currentTimeMillis();

        double sTime = System.currentTimeMillis();

        int count = 0;
        do {

            if (Robot.isTeleop || Robot.isDisabled) {
                throw new AutoInterruptedException();
            } else {
                error = (direction.value * degrees) - gyro.getAngle();
//                error = direction.value * ((Math.abs(degrees)/*90*/ > gyro.getAngle() ? (gyro.getAngle() - Math.abs(degrees)) : -((Math.abs(degrees)) - gyro.getAngle())));
//                integral += error * (System.currentTimeMillis() - prevTime);
//                prevTime = System.currentTimeMillis();
                power = kp * error/* + ki * integral*/;
                power = clip(power, -speed, +speed);
                setTarget(-power, -power, ControlMode.PercentOutput);

                if (Math.abs(error) < allowableError) {
                    count++;
                } else {
                    //count = 0;
                }
            }
            System.out.println("turning: " + degrees + ":" + direction.toString() + ":" + power);
        } while (count < 500 && System.currentTimeMillis() - sTime < timeKill);
        System.out.println("Completed");

        stopDrive();

    }

    public void turnP(double degrees, Direction direction, double speed, double allowableError, double killTime) throws AutoInterruptedException {
        resetGyro();

        double error;
        double power;
        double kp = 0.05;
        double integral = 0;
        double ki = 0.00003;
        double prevTime = System.currentTimeMillis();

        double sTime = System.currentTimeMillis();

        int count = 0;
        System.out.println(gyro.getAngle());
        do {

            if (Robot.isTeleop || Robot.isDisabled) {
                throw new AutoInterruptedException();
            } else {
                error = direction.value * ((Math.abs(degrees)/*90*/ < gyro.getAngle() ? (gyro.getAngle() - Math.abs(degrees)) : -((Math.abs(degrees)) - gyro.getAngle())));
                //error = direction.value * degrees - gyro.getAngle();
                System.out.print(error > 1 ? error + " " : "");
                integral += error * (System.currentTimeMillis() - prevTime);
                prevTime = System.currentTimeMillis();
                power = kp * error + ki * integral;
                power = clip(power, -speed, +speed);
                setTarget(power, power, ControlMode.PercentOutput);

                if (Math.abs(error) < allowableError) {
                    count++;
                } else {
                    count = 0;
                }
            }
        } while (count < 500 && System.currentTimeMillis() - sTime < killTime);
        System.out.println("Completed");
        stopDrive();
    }

    public static boolean robotStop = false;

    public void turnPID(double degrees, Direction direction, double speed) {
        resetGyro();

        double error = direction.value * degrees - gyro.getAngle();
        double prevError = error;
        double integral = 0;
        double derivative;
        double power;
        long dt;
        long prevTime = System.nanoTime();

        double kp = 0.025;
        double ki = 0.0;
        double kd = 0.0;
        double iDamper = 1.0;

        do {
            dt = System.nanoTime() - prevTime;
            prevTime = System.nanoTime();
            error = direction.value * degrees - gyro.getAngle();
            integral = iDamper * integral + error * dt;
            derivative = (error - prevError) / dt;
            prevError = error;
            power = kp * error + ki * integral + kd * derivative;
            power = clip(power, -speed, +speed);
            setTarget(-power, +power, ControlMode.PercentOutput);
        } while (Math.abs(error) > 0.5);
        stopDrive();
    }


    public void moveByGyroDistance(double inches, Direction direction, double speed, double allowableError, double timeKill) throws Exception {
        int targetClicks = (int) (inches * CLICKS_PER_INCH);
        int clicksRemaining;
        double inchesRemaining;
        double angularError;
        double dIntegral = 0;
        double powerAdjustment;
        double power;
        double leftPower;
        double rightPower;
        double maxPower;
        long dt;
        long prevTime = System.nanoTime();

        double prevAngle = 0;

        double p_gain = 0.05;
        double i_gain = 0.00005;
        double kp = 0.05;
        //double ki = 0.00005;
        //double kd = 0.0;
        //double iDamper = 1.0;
        resetGyro();
        resetEncoders();
        //int count = 0;
        double time = System.currentTimeMillis();
        System.out.println("moving");
        do {


            System.out.println("David 1");

            System.out.println("isDisabled = " + Robot.isDisabled);
            if (Robot.isDisabled || Robot.isTeleop) {
                System.out.println("David 2");
                throw new Exception();
            }

            clicksRemaining = targetClicks - Math.abs(rightTalon.getSensorCollection().getQuadraturePosition());
            inchesRemaining = clicksRemaining / CLICKS_PER_INCH;

            dt = System.nanoTime() - prevTime;
            prevTime = System.nanoTime();

            dIntegral += inchesRemaining * dt;

            power = inchesRemaining * p_gain + dIntegral * i_gain;
            power *= speed * direction.value;
            power = clip(power, -speed, speed);


            angularError = -gyro.getAngle();
            System.out.println("Angular Error: " + angularError);
            powerAdjustment = kp * angularError;
            powerAdjustment *= direction.value;
            powerAdjustment = clip(powerAdjustment, -1.0, +1.0);

            if (direction == Direction.BACKWARD) {
                leftPower = power + powerAdjustment;
                rightPower = power - powerAdjustment;
            } else {
                leftPower = power - powerAdjustment;
                rightPower = power + powerAdjustment;
            }
            maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

			/*
				angularIntegral = iDamper * angularIntegral + angularError * dt;
				angularDerivative = (angularError - prevAngularError) / dt;
				prevAngularError = angularError;
				powerAdjustment = kp * angularError + ki * angularIntegral + kd * angularDerivative;
			*/
            setTarget(-900 * leftPower, 900 * rightPower, ControlMode.Velocity);
			/*if(inchesRemaining < allowableError){
				count ++;
			}
			else{
				count = 0;
			}*/


        } while (inchesRemaining > 3 || angularError > 2 && System.currentTimeMillis() - time < timeKill);
        System.out.println("finished");
        stopDrive();
    }

    public double deadband(double input) {
        return Math.abs(input) < 0.15 ? 0.0 : input;
    }

    public void teleop(Joystick gamepad) {
        double left_y = deadband(gamepad.getRawAxis(LEFT_Y_AXIS));
        double right_y = deadband(gamepad.getRawAxis(RIGHT_Y_AXIS));
        double right_x = -deadband(gamepad.getRawAxis(RIGHT_X_AXIS));



        if (gamepad.getRawButton(BTN_BACK))
            driveMode = TeleopDriveModes.TANK_DRIVE;
        if (gamepad.getRawButton(BTN_START))
            driveMode = TeleopDriveModes.NEED_4_SPEED;

        if (false) {
            driveVelocity(-left_y, left_y);
        } else {
            // this.leftTalon.set(left_y * 500);
            // this.rightTalon.set(right_y * 500);
            double leftPower = left_y - right_x;
            double rightPower = left_y + right_x;

            if (Math.abs(leftPower) > 1) {
                leftPower /= Math.abs(leftPower);
                rightPower /= Math.abs(leftPower);
            }
            if (Math.abs(rightPower) > 1) {
                leftPower /= Math.abs(rightPower);
                rightPower /= Math.abs(rightPower);
            }

            //driveVbus(-leftPower, rightPower);
            driveVelocity(-leftPower * maxRPM, rightPower * maxRPM);
            // System.out.println("errL: " + leftTalon.getClosedLoopError(0) + " : errR:" + rightTalon.getClosedLoopError(0) + " : lSpeed: " + leftTalon.getSelectedSensorVelocity(0 ) + " : rSpeed : " + rightTalon.getSelectedSensorVelocity(0));
            /*
             * switch(driveMode) { case TANK_DRIVE: //driveVbus(left_y * 0.8,
             * right_y * 0.8); driveVelocity(-1250 * left_y, 1250 * right_y);
             * break; case NEED_4_SPEED: double leftPower = left_y - right_x;
             * double rightPower = left_y + right_x;
             *
             * if (Math.abs(leftPower) > 1) { leftPower /= Math.abs(leftPower);
             * rightPower /= Math.abs(leftPower); } if (Math.abs(rightPower) >
             * 1) { leftPower /= Math.abs(rightPower); rightPower /=
             * Math.abs(rightPower); }
             *
             *
             *
             * driveVelocity(1250 * leftPower, 1250 * rightPower); break; }
             */
        }
    }


    @Override
    protected void initDefaultCommand() {
        System.out.println("Minotaur DriveTrain");
    }

}
