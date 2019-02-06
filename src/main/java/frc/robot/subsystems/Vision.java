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

/**
 * Add your docs here.
 */
public class Vision extends Subsystem implements Constants, Section {
  private NetworkTable table;



  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void initDefaultCommand() {
    System.out.println("Vision started");
  }

  @Override
  public void teleop(MinoGamepad gamepad) {
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    System.out.println(x);
  }

  @Override
  public void reset() {

  }
}
