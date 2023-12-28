// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

// import com.ctre.phoenix.Logger;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    this.goalPose = goalSupplier.get();
    // this.poseEstimator.resetCounter(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("DriveToTarget/operating", true);
    ChassisSpeeds currentVelocity = swerve.getChassisSpeeds();
    Pose2d currentPose = poseEstimator.getCurrentPose();

    double distanceToTarget = goalPose.getTranslation().getDistance(currentPose.getTranslation());

    double directionToTarget = Math.atan2(
          goalPose.getY() - currentPose.getY(), 
          goalPose.getX() - currentPose.getX()
      );
    // System.out.println("speed: " + swerveDriveTrain.getSpeed());

    TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(3, 5)
     );
    double t = 0; // change

    var robot_chassis_speeds = swerve.getChassisSpeeds();
    var robot_velocity = Math.hypot(robot_chassis_speeds.vxMetersPerSecond, robot_chassis_speeds.vyMetersPerSecond);

    double velocity = profile.calculate(0.02,
      new TrapezoidProfile.State(distanceToTarget, 0),
      new TrapezoidProfile.State(0, robot_velocity)
    ).velocity;

    

    controlSpeeds.vxMetersPerSecond = Math.cos(directionToTarget) * velocity;
    controlSpeeds.vyMetersPerSecond = -Math.sin(directionToTarget) * velocity;

    
    Logger.recordOutput("NewDriveToTarget/RobotVelocity", velocity);
    Logger.recordOutput("NewDriveToTarget/x-velocity", controlSpeeds.vxMetersPerSecond);
    Logger.recordOutput("NewDriveToTarget/y-velocity", controlSpeeds.vyMetersPerSecond);
    
    Logger.recordOutput("NewDriveToTarget/x-error", (currentPose.getX() - goalPose.getX()));
    Logger.recordOutput("NewDriveToTarget/y-error", (currentPose.getY() - goalPose.getY()));

    // swerve.setWpiAbsoluteVelocoties(); // ???

  }
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
    } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
        newAngle += 360;
    }
    while (newAngle > upperBound) {
        newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
        newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
        newAngle += 360;
    }
    return newAngle;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controlSpeeds.vxMetersPerSecond = 0;
    controlSpeeds.vyMetersPerSecond = 0;

    //     swerveDriveTrain.setWpiAbsoluteVelocoties(controlSpeeds);
    Pose2d currentPose = this.poseEstimator.getCurrentPose();
    double xDistance = (goalPose.getX() - currentPose.getX());
    double yDistance = (goalPose.getY() - currentPose.getY());

    Logger.recordOutput("NewDriveToTarget/operating", false);

    // System.out.println("\n\n");
    // System.out.println("x: " + xDistance + " y: " + yDistance);




  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = poseEstimator.getCurrentPose();
    Pose2d goalPose = goalSupplier.get();

    double VxDistance = (goalPose.getX() - currentPose.getX());
    double VyDistance = (goalPose.getY() - currentPose.getY());

    double angleSetPoint = 0;
    double currentAngle = -swerve.getYawDegrees(); // In degrees.

    return Math.abs(VxDistance) < 0.05 && Math.abs(VyDistance) < 0.05 && Math.abs(angleSetPoint - currentAngle) < 1;
  }
}
