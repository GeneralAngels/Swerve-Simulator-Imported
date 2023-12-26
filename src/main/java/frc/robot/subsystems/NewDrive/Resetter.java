// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.NewDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;

public class Resetter extends TimeMeasurementSubsystem {
  /** Creates a new Resetter. */
  public Resetter() {}

  public void _periodic() {
    var modules = NewSwerveDriveSubsystem.getInstance().swerveModules;

    for (int i = 0; i < 4; i++) {
      modules[i].resetToAbsolute();
    }
  }
}
