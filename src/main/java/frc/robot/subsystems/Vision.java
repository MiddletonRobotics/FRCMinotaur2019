/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Utilities.Constants.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Utilities.Drivers.MinoGamepad;
import frc.robot.Utilities.Section;


public class Vision {
    private NetworkTable table;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public double getXPos() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0.0);
    }

    public boolean isTarget() {
        NetworkTableEntry tv = table.getEntry("tv");
        return (tv.getNumber(0).intValue() == 1)? true: false;
    }

    public void disableLight() {
        NetworkTableEntry light = table.getEntry("ledMode");
        light.setNumber(1);
    }

    public void enableLight() {
        NetworkTableEntry light = table.getEntry("ledMode");
        light.setNumber(3);
    }
}
