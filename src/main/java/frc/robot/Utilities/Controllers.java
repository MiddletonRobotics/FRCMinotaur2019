package frc.robot.Utilities;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import frc.robot.Utilities.Drivers.CKTalonSRX;
import frc.robot.Utilities.Drivers.CKVictorSPX;
import frc.robot.Utilities.Drivers.NavX;

import static frc.robot.Constants.*;

public class Controllers {
    private static Controllers instance = null;

    public static Controllers getInstance() {
        if (instance == null)
            instance = new Controllers();

        return instance;
    }

    private Controllers() {

        leftDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(masterLeftPort,0);
        leftDrive2 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(slaveLeftPort1, 1, leftDrive1);
        leftDrive3 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(slaveLeftPort2, 2, leftDrive1);
        rightDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(masterRightPort, 3);
        rightDrive2 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(slaveRightPort1, 4, rightDrive1);
        rightDrive3 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(slaveRightPort2, 5, rightDrive1);

        try {
            navX = new NavX(SPI.Port.kMXP);
        } catch (Exception ex) {
            //ConsoleReporter.report(ex, MessageLevel.DEFCON1);
        }

    }

    //Not sure if BaseMotorController or CKVictorSPX
    private CKTalonSRX leftDrive1;
    private BaseMotorController leftDrive2;
    private BaseMotorController leftDrive3;
    private CKTalonSRX rightDrive1;
    private BaseMotorController rightDrive2;
    private BaseMotorController rightDrive3;

    private NavX navX;

    public CKTalonSRX getLeftDrive1() { return leftDrive1; }

    public CKTalonSRX getRightDrive1() {
        return rightDrive1;
    }

    public BaseMotorController getLeftDrive2() {
        return leftDrive2;
    }

    public BaseMotorController getLeftDrive3() {
        return leftDrive3;
    }

    public BaseMotorController getRightDrive2() {
        return rightDrive2;
    }

    public BaseMotorController getRightDrive3() {
        return rightDrive3;
    }

    public NavX	getNavX() {
        return navX;
    }

}
