package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Drivers.MinoDoubleSol;
import frc.robot.Utilities.Section;


public class Intake extends Subsystem implements Section, Constants {

    private WPI_VictorSPX leftIntakeMotor;
    private WPI_VictorSPX rightIntakeMotor;
    private MinoDoubleSol rightIntakeSolenoid;
    private MinoDoubleSol leftIntakeSolenoid;

    private static Intake instance = null;

    private Intake() {
        leftIntakeMotor = new WPI_VictorSPX(intakeMotorRightID);
        rightIntakeMotor = new WPI_VictorSPX(intakeMotorLeftID);
        rightIntakeSolenoid = new MinoDoubleSol(rightIntakeSolenoidForwardChannel, rightIntakeSolenoidReverseChannel);
        leftIntakeSolenoid = new MinoDoubleSol(leftIntakeSolenoidForwardChannel, leftIntakeSolenoidReverseChannel);

        configTalon(leftIntakeMotor);
        configTalon(rightIntakeMotor);

        rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
        leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private void configTalon(WPI_VictorSPX motor) {
        motor.set(0);
        motor.configOpenloopRamp(0.25, 0);
    }
    @Override
    protected void initDefaultCommand() {
        System.out.println("yeah this thing sucks");
    }

    @Override
    public void teleop(Joystick gamepad) {
        if(gamepad.getRawAxis(RIGHT_T_AXIS) > 0) {
            setPercentSpeed(gamepad.getRawAxis(RIGHT_T_AXIS));
        } else {
            setPercentSpeed(gamepad.getRawAxis(LEFT_T_AXIS));
        }
    }

    @Override
    public void reset() {

    }

    public void setPercentSpeed(double speed) {
        rightIntakeMotor.set(speed);
        rightIntakeMotor.set(-speed);
    }
}
