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
import frc.robot.subsystems.Shooter;
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
    public Shooter shooter_subsystem = new Shooter();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    AutosGenerator autosGenerator = new AutosGenerator();

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

    public static List<PathPlannerPath> splitting_paths_into_segments(PathPlannerPath path_from_file) {
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
            PathPlannerPath path = PathPlannerPath.fromPathPoints(segment, segment.get(segment.size() / 2).constraints, new GoalEndState(0, segment.get(segment.size() - 1).holonomicRotation));

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
        return new RunCommand(() -> {
            NewSwerveDriveSubsystem.getInstance().setAbsoluteVelocities(new ChassisSpeeds(1, 1, 0.2));
        }).withTimeout(5);
    }
}
