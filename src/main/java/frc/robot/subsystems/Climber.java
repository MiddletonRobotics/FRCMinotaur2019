package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.Robot;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Constants.Positions.ClimberPositions;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.PIDController;
import frc.robot.Utilities.Section;

public class Climber implements Section, Constants {

    private WPI_TalonSRX climberFrontMotor;
    private WPI_TalonSRX climberRearMotor;
    private WPI_TalonSRX rearClimberDrive;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);


    private DigitalInput frontLimitSwitch;
    private DigitalInput rearLimitSwitch;

    private static Climber instance = null;
    private int position = 0;
    private boolean lifting = false;
    private boolean resetting = false;
    private boolean startPrevious = false;
    public PIDController frontPID = new PIDController(kClimberFrontKp, kClimberFrontKi, kClimberFrontKd, 0);
    public PIDController rearPID = new PIDController(kClimberRearKp, kClimberRearKi, kClimberRearKd, 0);



    private Climber() {
        climberFrontMotor = new WPI_TalonSRX(climberFrontID);
        climberRearMotor = new WPI_TalonSRX(climberRearID);
        rearClimberDrive = new WPI_TalonSRX(rearClimberDriveID);
        frontLimitSwitch = new DigitalInput(frontLimitSwitchPort);
        rearLimitSwitch = new DigitalInput(rearLimitSwitchPort);


        configFrontTalon(climberFrontMotor);
        configMiscTalon(climberRearMotor);
        configMiscTalon(rearClimberDrive);

        frontPID.setSetpoint(getFrontPosition());
        /*rearPID.setSetpoint(getRearPosition());*/
        frontPID.enable();
    }

    public static Climber getInstance() {
        if(instance == null) {
            instance = new Climber();
        }

        return instance;
    }


    public double getFrontPosition() {
        return climberFrontMotor.getSensorCollection().getQuadraturePosition();
    }

    private void configFrontTalon(WPI_TalonSRX motor) {
        motor.set(0);

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
        motor.configAllowableClosedloopError(0, 0, kTimeoutMs);

        motor.configForwardSoftLimitEnable(false, kTimeoutMs);
        motor.configReverseSoftLimitEnable(false, kTimeoutMs);

        motor.setNeutralMode(NeutralMode.Brake);
    }

    private void configMiscTalon(WPI_TalonSRX motor) {
        motor.set(0);
        motor.configForwardSoftLimitEnable(false, kTimeoutMs);
        motor.configReverseSoftLimitEnable(false, kTimeoutMs);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void initDefaultCommand() {
        System.out.println("lebel tree hehe");
    }


    @Override
    public void teleop(MinoGamepad gamepad) {
/*

        System.out.println(frontPID.getError());

        if (gamepad.start()) {
            if (startPrevious) {
                lifting = !lifting;
            }
        }



        if (lifting) {
            frontPID.enable();
            rearPID.enable();
            frontPID.setSetpoint(ClimberPositions.frontClimberOut);
            rearPID.setSetpoint();
            usePIDOutput();
            rearClimberDrive.set(ControlMode.PercentOutput, gamepad.leftStickY());
        } else {
            frontPID.enable();
            rearPID.enable();
            frontPID.setSetpoint(ClimberPositions.frontClimberOut);
            rearPID.setSetpoint();
            usePIDOutput();
            rearClimberDrive.set(ControlMode.PercentOutput, 0);

        }

        if (gamepad.rightTriggerPressed()) {
            setClimberFrontSpeedPercent(0.1);
        } else if (gamepad.leftTriggerPressed()) {
            setClimberFrontSpeedPercent(-0.1);
        } else {
            stopClimber();
        }

        if (gamepad.rightBumper()) {
            setClimberRearSpeedPercent(0.1);
        } else if (gamepad.leftBumper()) {
            setClimberRearSpeedPercent(-0.1);
        } else {
            stopClimber();
        }
*/

    }

/*    private void stopPID() {
        *//*if (manual) {
            frontPID.setSetpoint(climberFrontMotor.getSensorCollection().getQuadraturePosition());
        }
*//**//*
        frontPID.setF(kClimberKf);
*//*
        frontPID.enable();
        rearPID.enable();
        usePIDOutput();

    }*/

    public boolean getLifting() {
        return lifting;
    }

    public boolean getResetting() {
        return resetting;
    }

    private AHRS getGyro() {
        return gyro;
    }

    public void resetGyro() {
        gyro.reset();

    }

    @Override
    public void reset() {
/*
        zeroClimber();
*/
    }

/*    public void zeroClimber() {

        climberFrontMotor.set(ControlMode.Disabled, 0);
        int homeClimberValue = (int)(ClimberPositions.climberHomePosition * kClimberEncoderGearRatio * sensorUnitsPerRotationMag);

        climberFrontMotor.setSelectedSensorPosition(homeClimberValue, 0, Constants.kTimeoutMs);
        climberFrontMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        climberFrontMotor.set(ControlMode.MotionMagic, homeClimberValue);

    }*/

    public void resetEnoders() {
        climberFrontMotor.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
        climberRearMotor.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    }

    public void zeroClimber() {
        new Thread( () -> {
            resetting = true;
            long startTime = System.currentTimeMillis();
            while ((frontLimitSwitch.get() || rearLimitSwitch.get()) && System.currentTimeMillis() - startTime < 3000) {
                setClimberFrontSpeedPercent(frontLimitSwitch.get()? -0.2 : 0);
                setClimberRearSpeedPercent(rearLimitSwitch.get()? -0.2 : 0);
            }
            stopClimber();
            resetEnoders();
            resetting = false;
            frontPID.setSetpoint(ClimberPositions.frontClimberHome);
        }).start();
    }


    protected void usePIDOutput() {
        setClimberRearSpeedPercent(-frontPID.get(getFrontPosition()));
        setClimberRearSpeedPercent(-rearPID.get(gyro.getAngle()));
    }

    public void stopClimber() {
        climberFrontMotor.set(0);
        climberRearMotor.set(0);
    }

    public void setClimberFrontSpeedPercent(double speed) {

        climberFrontMotor.set(ControlMode.PercentOutput, speed);

    }

    public void setClimberRearSpeedPercent(double speed) {

        climberRearMotor.set(ControlMode.PercentOutput, speed);

    }

    public WPI_TalonSRX getClimberFrontMotor() {
        return climberFrontMotor;
    }

    public WPI_TalonSRX getClimberRearMotor() {
        return climberRearMotor;
    }

}
