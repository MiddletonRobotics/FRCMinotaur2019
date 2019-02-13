package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Robot;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Section;

public class DriveTrain extends Subsystem implements Constants, Section {

    private final WPI_TalonSRX leftTalon, rightTalon;
    private final WPI_VictorSPX leftSlave, rightSlave;
    private final WPI_VictorSPX leftSlave2, rightSlave2;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private static DriveTrain instance = null;

    private DriveTrain() {
        leftTalon = new WPI_TalonSRX(leftDrivetrainMasterID);
        leftSlave = new WPI_VictorSPX(leftDrivetrainSlave1ID);
        leftSlave2 = new WPI_VictorSPX(leftDrivetrainSlave2ID);

        rightTalon = new WPI_TalonSRX(rightDrivetrainMasterID);
        rightSlave = new WPI_VictorSPX(rightDrivetrainSlave1ID);
        rightSlave2 = new WPI_VictorSPX(rightDrivetrainSlave2ID);

        setupSlaves(leftTalon, leftSlave);
        setupSlaves(leftTalon, leftSlave2);
        setupSlaves(rightTalon, rightSlave);
        setupSlaves(rightTalon, rightSlave2);

        configTalon(leftTalon, true);
        configTalon(rightTalon, true);

        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP
        TalonHelper.setPIDGains(leftTalon, 0, kpLeftDriveVel, kiLeftDriveVel, kdLeftDriveVel, kfLeftDriveVel, 0, 0); // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP
        TalonHelper.setPIDGains(rightTalon, 0, kpRightDriveVel, kiRightDriveVel, kdRightDriveVel, kfRightDriveVel, 0, 0); // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP
        /*TalonHelper.setPIDGains(master, 1, kpDriveTrainPos, kiDriveTrainPos, kdDriveTrainPos, kfDriveTrainPos, 0, 0);
        TalonHelper.setPIDGains(master, 2, kpDriveTrainPos2, kiDriveTrainPos2, kdDriveTrainPos2, kfDriveTrainPos2, 0, 0);*/
        // HEY YOU HAVE TO EDIT THE IZONE FROM ZERO FOR INTEGRAL WINDUP
        initializeVariables();
    }

    public static DriveTrain getInstance() {
        if (instance == null) {
            instance = new DriveTrain();
        }

        return instance;
    }


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

    @Override
    public void reset() {
        leftTalon.set(0);
        rightTalon.set(0);
        resetEncoders();
        resetGyro();
        setProfile(0);

    }

    private void setupSlaves(WPI_TalonSRX master, WPI_VictorSPX slave) {
        slave.follow(master);
    }

    private void configTalon(WPI_TalonSRX master, boolean reversed) {
        master.set(0);

        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        master.setSensorPhase(reversed);
        // master.configClosedloopRamp(0.25, 0);
        master.configOpenloopRamp(0.25, 0);
//      masteqr.configNominalOutputForward(NOMINAL_OUTPUT_VOLTAGE, -NOMINAL_OUTPUT_VOLTAGE);
        //    master.configPeakOutputVoltage(+PEAK_OUTPUT_VOLTAGE, -PEAK_OUTPUT_VOLTAGE);

        //aster.configSen R);

        master.configAllowableClosedloopError(0, 0, 0);

        //  master.setVoltageRampRate(48);

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
        return leftTalon.getSensorCollection().getQuadratureVelocity()/NATIVE_PER_ROTATION;
    }

    public double getRightVelocity() {
        return rightTalon.getSensorCollection().getQuadratureVelocity()/NATIVE_PER_ROTATION;
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

    public double inchesToRotations(double inches) {
        return inches / (WHEEL_DIAMETER * Math.PI);
    }


    public static boolean robotStop = false;

    public void turnPID(double degrees, Direction direction, double speed) {
        resetGyro();

        double error = direction.value * degrees - gyro.getAngle();
        double prevError = error;
        double integral = 0;
        long prevTime = System.nanoTime();

        while (Math.abs(error) > angleTolerance) {
            long dt = System.nanoTime() - prevTime;
            prevTime = System.nanoTime();
            error = direction.value * degrees - gyro.getAngle();
            integral = turnIDamper * integral + error * dt;
            double derivative = (error - prevError) / dt;
            prevError = error;
            double power = turnKp * error + turnKi * integral + turnKd * derivative;
            power = clip(power, -speed, +speed);
            setTarget(-power, +power, ControlMode.PercentOutput);
        } 
        stopDrive();
    }



    private double prevErrorTPOM;
    private double integralTPOM;
    private long prevTimeTPOM;
    private boolean firstRunTPOM = true;
    public boolean turnProportionalOnMeasurement(double degrees, Direction direction) {
        if (firstRunTPOM) {
            initializeVariables();
            resetGyro();
            prevTimeTPOM = System.nanoTime();
            firstRunTPOM = !firstRunTPOM;
        }

        double errorTPOM = direction.value * degrees - gyro.getAngle();
        if (Math.abs(errorTPOM) > angleTolerance? true: gyro.getRate() > ROBOT_THRESHOLD_DEGREES_PER_SECOND? true: false) {
            long dt = System.nanoTime() - prevTimeTPOM;
            double measurementTPOM = gyro.getAngle();
            integralTPOM += gyro.getRate() <= ROBOT_MAX_DEGREES_PER_SECOND_INTEGRAL_LIMIT? errorTPOM * dt : 0;
            double derivative = (errorTPOM - prevErrorTPOM) / dt;
            prevErrorTPOM = errorTPOM;
            double velocity = -turnPOMKp * measurementTPOM + turnPOMKi * integralTPOM + turnPOMKd * derivative;
            velocity = -clip(velocity, -MAX_VELOCITY_NATIVE, MAX_VELOCITY_NATIVE);
            setTarget(velocity, velocity, ControlMode.Velocity);
            prevTimeTPOM = System.nanoTime();
            return false;
        } else {
            stopDrive();
            return true;
        }

    }

    private double prevErrorMGDPOM;
    private long prevTimeMGDPOM;
    private boolean firstRunMGDPOM = true;
    private double distanceIntegralMGDPOM = 0;
    private double angleIntegralMGDPOM = 0;
    public boolean moveGyroDistancePOM(double inches, Direction direction,  double allowableError, double timeKill) {
        if (firstRunMGDPOM) {
            initializeVariables();
            resetGyro();
            resetEncoders();
            prevTimeMGDPOM = System.nanoTime();
            firstRunMGDPOM = !firstRunMGDPOM;
        }

        int targetClicks = (int) (inches * CLICKS_PER_INCH);
        int clicksRemaining = targetClicks - Math.abs(rightTalon.getSensorCollection().getQuadraturePosition());

        double inchesRemaining = clicksRemaining / CLICKS_PER_INCH;
        double angularError = gyro.getAngle();


        double time = System.currentTimeMillis();
        // ADDDDDD MIN SPEED THRESHOLD

        if (Math.abs(inchesRemaining) > distanceTolerance || Math.abs(angularError) > angleTolerance && System.currentTimeMillis() - time < timeKill) {

            long dt = System.nanoTime() - prevTimeMGDPOM;

            double measurement = Math.abs(rightTalon.getSensorCollection().getQuadraturePosition())/CLICKS_PER_INCH;

            distanceIntegralMGDPOM += inchesRemaining * dt;
            angleIntegralMGDPOM += angularError * dt;

            double speed = direction.value * (-measurement * gyroDrivePOMKP + distanceIntegralMGDPOM * gyroDrivePOMKI);
            speed = clip(speed, -MAX_VELOCITY_NATIVE, MAX_VELOCITY_NATIVE);

            double powerAdjustment = gyroCorrectionKP * angularError + angleIntegralMGDPOM * gyroCorrectionKI;
            powerAdjustment = clip(powerAdjustment, -MAX_VELOCITY_NATIVE, MAX_VELOCITY_NATIVE);

            double leftSpeed = speed + powerAdjustment;
            double rightSpeed = speed + powerAdjustment;

            double maxPower = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

            if (maxPower > MAX_VELOCITY_NATIVE) {
                leftSpeed *= (MAX_VELOCITY_NATIVE/maxPower);
                rightSpeed *= (MAX_VELOCITY_NATIVE/maxPower);
            }

            setTarget(leftSpeed, -rightSpeed, ControlMode.Velocity);
            prevTimeMGDPOM = System.nanoTime();

            return true;
        } else {
            stopDrive();
            return true;
        }
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
                //throw new Exception();
                break;
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

    public void teleop(MinoGamepad gamepad) {
        //System.out.println("right: " + rightTalon.getSensorCollection().getQuadratureVelocity());
        //System.out.println("left: " + leftTalon.getSensorCollection().getQuadratureVelocity());
        double left_y = deadband(gamepad.getRawAxis(LEFT_Y_AXIS));
        double right_x = deadband(gamepad.getRawAxis(RIGHT_X_AXIS));

        if (gamepad.getRawButton(BTN_BACK))
            driveMode = TeleopDriveModes.TANK_DRIVE;
        if (gamepad.getRawButton(BTN_START))
            driveMode = TeleopDriveModes.NEED_4_SPEED;


        double leftPower = left_y - right_x;
        double rightPower = left_y + right_x;


        double maxSpeed = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxSpeed > 1) {
            leftPower /= maxSpeed;
            rightPower /= maxSpeed;
        }

        if (gamepad.a()) {
            leftTalon.set(ControlMode.Velocity, /*-maxNativeVelocity*leftPower*/1000);
            rightTalon.set(ControlMode.Velocity, /*maxNativeVelocity*rightPower*/-1000);
        } else {
            leftTalon.set(ControlMode.PercentOutput, -maxNativeVelocity*leftPower);
            rightTalon.set(ControlMode.PercentOutput, maxNativeVelocity*rightPower);
        }

    }



    public void initializeVariables() {
        prevErrorTPOM = 0;
        integralTPOM = 0;
        prevTimeTPOM = 0;
        firstRunTPOM = true;

        prevErrorMGDPOM = 0;
        prevTimeMGDPOM = 0;
        firstRunMGDPOM = true;
        distanceIntegralMGDPOM = 0;
        angleIntegralMGDPOM = 0;

        // prevError = 0;
        // integral = 0;
        // prevTime = 0;
        // firstRun = true;
    }

    @Override
    protected void initDefaultCommand() {
        System.out.println("Minotaur DriveTrain");
    }

}
