// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

public class DriveToTarget extends Command {
  private Supplier<Pose2d> cameraPoseSupplier;
  private NewPoseEstimatorSubsystem poseEstimator;
  private NewSwerveDriveSubsystem swerve;
  private Pose2d cameraPose;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(Supplier<Pose2d> cameraPose) {
    this.cameraPoseSupplier = cameraPose;
    this.poseEstimator = NewPoseEstimatorSubsystem.getInstance();
    this.swerve = NewSwerveDriveSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.cameraPose = cameraPoseSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
