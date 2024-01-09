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

  /** Creates a new DriveToTarget. */
  public DriveToTarget(Supplier<Pose2d> goalSupplier) {
    // this.cameraPoseSupplier = cameraPose;
    this.poseEstimator = NewPoseEstimatorSubsystem.getInstance();
    this.swerve = NewSwerveDriveSubsystem.getInstance(); 
    this.goalSupplier = goalSupplier;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.cameraPose = cameraPoseSupplier.get();
    this.goalPose = new Pose2d(1.87, 3.82, Rotation2d.fromDegrees(180));
    System.out.println("\n\n hello world"); 
    // this.poseEstimator.resetCounter(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("DriveToTarget/operating", true);
    ChassisSpeeds currentVelocity = swerve.getChassisSpeeds();
    Pose2d currentPose = poseEstimator.getCurrentPose();

    ProfiledPIDController profileX = new ProfiledPIDController(0, 0, 0, new Constraints(3, 5));
    ProfiledPIDController profileY = new ProfiledPIDController(0, 0, 0, new Constraints(3, 5));


    // double distanceToTarget = goalPose.getTranslation().getDistance(currentPose.getTranslation());
    // System.out.println("speed: " + swerveDriveTrain.getSpeed());

    // TrapezoidProfile profileX = new TrapezoidProfile(
    //   new TrapezoidProfile.Constraints(3, 5)
    //  );
    // TrapezoidProfile profileY = new TrapezoidProfile(
    //   new TrapezoidProfile.Constraints(3, 5)
    //  );

    var robot_chassis_speeds = swerve.getChassisSpeeds();
    // var robot_velocity = Math.hypot(robot_chassis_speeds.vxMetersPerSecond, robot_chassis_speeds.vyMetersPerSecond);
    
    // double directionToTarget = Math.atan2(
    //       goalPose.getY() - currentPose.getY(), 
    //       goalPose.getX() - currentPose.getX()
    //   );

    profileX.reset(0, robot_chassis_speeds.vxMetersPerSecond);
    double velocityX = profileX.calculate(currentPose.getX(), new State(goalPose.getX(), 0)
    );

    profileY.reset(0, robot_chassis_speeds.vxMetersPerSecond);
    double velocityY = profileY.calculate(currentPose.getY(),
      new TrapezoidProfile.State(goalPose.getY(), 0)
    );


    controlSpeeds.vxMetersPerSecond = velocityX;
    controlSpeeds.vyMetersPerSecond = velocityY;
    
    // Logger.recordOutput("NewDriveToTarget/RobotVelocity", velocity);
    Logger.recordOutput("NewDriveToTarget/x-velocity", controlSpeeds.vxMetersPerSecond);
    Logger.recordOutput("NewDriveToTarget/y-velocity", controlSpeeds.vyMetersPerSecond);
    
    Logger.recordOutput("NewDriveToTarget/x-error", (currentPose.getX() - goalPose.getX()));
    Logger.recordOutput("NewDriveToTarget/y-error", (currentPose.getY() - goalPose.getY()));


    swerve.setAbsoluteVelocities(controlSpeeds);

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

    System.out.println("\n\n");
    System.out.println("x: " + xDistance + " y: " + yDistance);




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

    if (Math.abs(VxDistance) < 0.05 && Math.abs(VyDistance) < 0.05 && Math.abs(angleSetPoint - currentAngle) < 1) System.out.println("\n\n finished");
    return Math.abs(VxDistance) < 0.05 && Math.abs(VyDistance) < 0.05 && Math.abs(angleSetPoint - currentAngle) < 1;
  }
}
