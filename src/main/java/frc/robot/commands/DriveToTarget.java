// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

// import com.ctre.phoenix.Logger;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

public class DriveToTarget extends Command {
  private Supplier<Pose2d> cameraPoseSupplier;
  private NewPoseEstimatorSubsystem poseEstimator;
  private NewSwerveDriveSubsystem swerve;
  private Pose2d cameraPose;
  private Supplier<Pose2d> goalSupplier;
  private Pose2d goalPose;
  ChassisSpeeds controlSpeeds = new ChassisSpeeds();

  ProfiledPIDController profileX = new ProfiledPIDController(2, 0, 0, new Constraints(3, 5));
  ProfiledPIDController profileY = new ProfiledPIDController(2, 0, 0, new Constraints(3, 5));
  ProfiledPIDController profileRot = new ProfiledPIDController(2, 0, 0.2, new Constraints(3, 5));
  
  
  /** Creates a new DriveToTarget. */
  public DriveToTarget(Supplier<Pose2d> goalSupplier) {
    // this.cameraPoseSupplier = cameraPose;
    this.poseEstimator = NewPoseEstimatorSubsystem.getInstance();
    this.swerve = NewSwerveDriveSubsystem.getInstance(); 
    this.goalSupplier = goalSupplier;
    // Use addRequirements() here to declare subsystem dependencies.

    profileRot.enableContinuousInput(-Math.PI, Math.PI);

    profileX.setGoal(goalSupplier.get().getX());
    profileY.setGoal(goalSupplier.get().getY());
    profileRot.setGoal(goalSupplier.get().getRotation().getRadians());

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.cameraPose = cameraPoseSupplier.get();
    this.goalPose = goalSupplier.get();
    System.out.println("\n\n hello world"); 
    // this.poseEstimator.resetCounter(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("DriveToTarget/operating", true);
    ChassisSpeeds currentVelocity = swerve.getChassisSpeeds(); 
    Pose2d currentPose = poseEstimator.getCurrentPose();
    
    controlSpeeds.vxMetersPerSecond = profileX.calculate(currentPose.getX());
    controlSpeeds.vyMetersPerSecond = profileY.calculate(currentPose.getY());
    
    controlSpeeds.omegaRadiansPerSecond = profileRot.calculate(currentPose.getRotation().getRadians());
    // controlSpeeds.vxMetersPerSecond = goalPose.getX() - currentPose.getX();
    // controlSpeeds.vyMetersPerSecond = goalPose.getY() - currentPose.getY();

    // Logger.recordOutput("NewDriveToTarget/RobotVelocity", velocity);a
    Logger.recordOutput("NewDriveToTarget/x-velocity", controlSpeeds.vxMetersPerSecond);
    Logger.recordOutput("NewDriveToTarget/y-velocity", controlSpeeds.vyMetersPerSecond);
    Logger.recordOutput("NewDriveToTarget/R-velocity", controlSpeeds.omegaRadiansPerSecond);
    
    Logger.recordOutput("NewDriveToTarget/x-error", -(currentPose.getX() - goalPose.getX()));
    Logger.recordOutput("NewDriveToTarget/y-error", -(currentPose.getY() - goalPose.getY()));
    Logger.recordOutput("NewDriveToTarget/r-error", -(currentPose.getRotation().getRadians() - goalPose.getRotation().getRadians()));

  
    Logger.recordOutput("Current Radians", currentPose.getRotation().getRadians());

    swerve.setAbsoluteVelocities(controlSpeeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controlSpeeds.vxMetersPerSecond = 0;
    controlSpeeds.vyMetersPerSecond = 0;
    controlSpeeds.omegaRadiansPerSecond = 0;

    //     swerveDriveTrain.setWpiAbsoluteVelocoties(controlSpeeds);
    Pose2d currentPose = this.poseEstimator.getCurrentPose();
    double xDistance = (goalPose.getX() - currentPose.getX());
    double yDistance = (goalPose.getY() - currentPose.getY());
    double rDistance = (goalPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());


    Logger.recordOutput("NewDriveToTarget/operating", false);

    System.out.println("\n\n");
    System.out.println("x: " + xDistance + " y: " + yDistance + " R: " + rDistance);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = poseEstimator.getCurrentPose();
    Pose2d goalPose = goalSupplier.get();

    double VxDistance = (goalPose.getX() - currentPose.getX());
    double VyDistance = (goalPose.getY() - currentPose.getY());
    double VRDistance = (goalPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());

    double angleSetPoint = 0;
    double currentAngle = -swerve.getYawDegrees(); // In degrees.

    if (Math.abs(VxDistance) < 0.05 && Math.abs(VyDistance) < 0.05 && Math.abs(angleSetPoint - currentAngle) < 1) System.out.println("\n\n finished");
    return Math.abs(VxDistance) < 0.05 && Math.abs(VyDistance) < 0.05 && Math.abs(angleSetPoint - currentAngle) < 1;
  }
}
