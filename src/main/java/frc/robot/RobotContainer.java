// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;
import frc.robot.subsystems.PoseEstimator;
import com.pathplanner.lib.util.*;
import com.pathplanner.lib.commands.*;
import org.photonvision.PhotonCamera;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    public NewSwerveDriveSubsystem newSwerve = NewSwerveDriveSubsystem.getDefaultSwerve();

    public NewPoseEstimatorSubsystem poseEstimatorSubsystem = new NewPoseEstimatorSubsystem(new PhotonCamera(""), newSwerve);

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    public void func(Supplier<Double> supplier) {
        System.out.println(supplier.get());
    }

    public List<PathPlannerPath> splitting_paths_into_segments(PathPlannerPath path_from_file) {
        System.out.println(path_from_file);

        // Splitting path into segments:
        List<List<PathPoint>> segments_list = new ArrayList<>();
        segments_list.add(new ArrayList<PathPoint>());
        int segment_index = 0;

        for (PathPoint pathPoint : path_from_file.getAllPathPoints()) {
            segments_list.get(segment_index).add(pathPoint);

            if (pathPoint.holonomicRotation != null) {
                System.out.println("x: " + pathPoint.position.getX() + ", y: " + pathPoint.position.getY());
                System.out.println("degrees: " + pathPoint.holonomicRotation.getDegrees());
                System.out.println();

                segment_index += 1;
                
                System.out.println("segment index: " + segment_index);
                segments_list.add(new ArrayList<PathPoint>());
                segments_list.get(segment_index).add(pathPoint);
            }
        }

        double totalTime = 0;

        System.out.println(segments_list.toString());

        ArrayList<PathPlannerTrajectory> pathPlannerTrajectories = new ArrayList<PathPlannerTrajectory>();
        ArrayList<PathPlannerPath> pathList = new ArrayList<PathPlannerPath>();

        for (List<PathPoint> segment : segments_list) {
            if (segment.size() <= 1) {
                continue;
            }
            System.out.println("segment length: " + segment.size());
            PathPlannerPath path = PathPlannerPath.fromPathPoints(
                    segment,
                    path_from_file.getGlobalConstraints(),
                    new GoalEndState(0, segment.get(segment.size() - 1).holonomicRotation));

            pathList.add(path);

            PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds());

            pathPlannerTrajectories.add(trajectory);

            System.out.println(trajectory.getTotalTimeSeconds());
            totalTime += trajectory.getTotalTimeSeconds();
        }

        System.out.println();
        System.out.println("path list length: " + pathList.size());
        System.out.println();

        return pathList;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        var wanted_speeds = new ChassisSpeeds(0.2, 1, Units.degreesToRadians(20));

        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Example Path");

        List<PathPlannerPath> paths = splitting_paths_into_segments(path_from_file);

        poseEstimatorSubsystem.setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());

        if (Robot.isSimulation()) {
            newSwerve.pigeonSimCollection.setRawHeading(0);
        }
        newSwerve.pigeon2.setYaw(0);

        var path_planner_command = new FollowPathHolonomic(
                paths.get(0),
                poseEstimatorSubsystem::getCurrentPose, // Robot pose supplier
                newSwerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                newSwerve::setRelativeVelocities, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.91, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                newSwerve // Reference to this subsystem to set requirements
        ).andThen(
                new FollowPathHolonomic(
                        paths.get(1),
                        poseEstimatorSubsystem::getCurrentPose, // Robot pose supplier
                        newSwerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        newSwerve::setRelativeVelocities, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                                4.5, // Max module speed, in m/s
                                0.91, // Drive base radius in meters. Distance from robot center to furthest module.
                                new ReplanningConfig() // Default path replanning config. See the API for the options here
                        ),
                        newSwerve // Reference to this subsystem to set requirements
            )
        );

        /*
        return new RunCommand(
                () -> {newSwerve.setAbsoluteVelocities(wanted_speeds);},
                newSwerve
        ).withTimeout(30).andThen(new InstantCommand(() -> {newSwerve.setRelativeVelocities(new ChassisSpeeds());}, newSwerve));
         */

        return path_planner_command;

    }
}
