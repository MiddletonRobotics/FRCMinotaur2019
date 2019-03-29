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
    private AHRS gyro = new AHRS(SPI.Port.kMXP);


    private DigitalInput limitSwitch;

    private static Climber instance = null;
    private int position = 0;
    private boolean lifting = false;
    private boolean resetting = false;
    public PIDController frontPID = new PIDController(kClimberFrontKp, kClimberFrontKi, kClimberFrontKd, 0);
    public PIDController rearPID = new PIDController(kClimberRearKp, kClimberRearKi, kClimberRearKd, 0);



    private Climber() {
        climberFrontMotor = new WPI_TalonSRX(climberFrontID);
        climberRearMotor = new WPI_TalonSRX(climberRearID);


        configTalon(climberFrontMotor);
        configTalon(climberRearMotor);

/*
        frontPID.setSetpoint(getFrontPosition());
*/
        rearPID.setSetpoint(getRearPosition());
        frontPID.enable();
    }

    public static Climber getInstance() {
        if(instance == null) {
            instance = new Climber();
        }

        return instance;
    }


    public double getRearPosition() {
        return climberRearMotor.getSensorCollection().getQuadraturePosition();
    }

    private void configTalon(WPI_TalonSRX motor) {
        motor.set(0);

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
        motor.setSensorPhase(true);
        motor.setInverted(true);

        motor.configAllowableClosedloopError(0, 0, kTimeoutMs);


        motor.configForwardSoftLimitEnable(false, kTimeoutMs);
        motor.configReverseSoftLimitEnable(false, kTimeoutMs);

        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void initDefaultCommand() {
        System.out.println("lebel tree hehe");
    }


    @Override
    public void teleop(MinoGamepad gamepad) {


        System.out.println(frontPID.getError());

        if (gamepad.y()) {

        } else {
            stopPID();
        }


    }

    private void stopPID() {
        /*if (manual) {
            frontPID.setSetpoint(climberFrontMotor.getSensorCollection().getQuadraturePosition());
        }
*//*
        frontPID.setF(kClimberKf);
*/
        frontPID.enable();
        rearPID.enable();
        usePIDOutput();

    }

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
        /*zeroClimber();*/
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
/*
    public void zeroClimber() {
        new Thread( () -> {
            resetting = true;
            long startTime = System.currentTimeMillis();
            while (limitSwitch.get() && System.currentTimeMillis() - startTime < 3000) {
                setClimberSpeedPercent(-0.2);
            }
            stopClimber();
            resetEnoder();
            resetting = false;
        }).start();
    }*/
    
    
    protected void usePIDOutput() {
        if (!resetting) {
/*
            setClimberFrontSpeedPercent(-frontPID.get(getFrontPosition()));
*/
            setClimberRearSpeedPercent(-rearPID.get(getRearPosition()));
        }
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
