package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants.Constants;
import frc.robot.Utilities.Drivers.MinoDoubleSol;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Section;


public class Intake extends Subsystem implements Section, Constants {

    private WPI_TalonSRX rightIntakeMaster;
    private WPI_VictorSPX leftIntakeSlave;
    public MinoDoubleSol intakeSolenoid;


    private static Intake instance = null;

    private Intake() {
        rightIntakeMaster = new WPI_TalonSRX(intakeMasterRightID);
        leftIntakeSlave = new WPI_VictorSPX(intakeSlaveLeftID);
        intakeSolenoid = new MinoDoubleSol(intakeSolenoidForwardChannel, intakeSolenoidReverseChannel);

        configTalons();

        closeIntake();
    }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private void configTalons() {
        rightIntakeMaster.set(0);
        leftIntakeSlave.set(0);
        rightIntakeMaster.configOpenloopRamp(0, 0);
        leftIntakeSlave.configOpenloopRamp(0, 0);
        leftIntakeSlave.setNeutralMode(NeutralMode.Brake);
        rightIntakeMaster.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    protected void initDefaultCommand() {
        System.out.println("yeah this thing sucks");
    }

    private boolean intaking = false;
    @Override
    public void teleop(MinoGamepad gamepad) {


        if(gamepad.leftTriggerPressed()) {
            intaking = false;
            rightIntakeMaster.configOpenloopRamp(0, kTimeoutMs);
            leftIntakeSlave.configOpenloopRamp(0, kTimeoutMs);
            setPercentSpeed(intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ? -1 : 1); //OUT
        } else if (gamepad.rightTriggerPressed()){
            intaking = true;
            rightIntakeMaster.configOpenloopRamp(0.25, kTimeoutMs);
            leftIntakeSlave.configOpenloopRamp(0.25, kTimeoutMs);
            setPercentSpeed(intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ? 1 : -1); //IN
        } else {
            if (intaking) {
                setPercentSpeed(intakeSolenoid.getValue() == DoubleSolenoid.Value.kForward ? 0.05 : -0.05); //IN
            } else {
                setPercentSpeed(0);
            }

        }

        if (gamepad.getRawButton(BTN_X)) {
            toggleIntake();
        }

    }


    public void openIntake() {
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void closeIntake() {
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void toggleIntake() {
        intakeSolenoid.toggle();
    }

    @Override
    public void reset() {
        rightIntakeMaster.set(0);
        leftIntakeSlave.set(0);
        closeIntake();
    }

    public void setPercentSpeed(double speed) {
        rightIntakeMaster.set(-speed);
        leftIntakeSlave.set(speed);
    }
}
