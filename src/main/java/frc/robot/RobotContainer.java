// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    public Shooter_Test_RigSubsystem shooter_rig = new Shooter_Test_RigSubsystem();

    AutosGenerator autosGenerator;

    CommandPS4Controller driver = new CommandPS4Controller(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        AmpScorer.getInstance();
        ClimbingSubsystem.getInstance();
        // Configure the button bindings
        configureButtonBindings();

        // To initialize all subsystems:
        NewSwerveDriveSubsystem.getInstance();
        if (Robot.isReal()) {
            NewSwerveDriveSubsystem.getInstance().pigeon2.setYaw(0);
            System.out.println("setting yaw " );
        }

        NewPoseEstimatorSubsystem.getInstance();

        this.autosGenerator = new AutosGenerator();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // CLIMB
        driver.cross().toggleOnTrue(
            new InstantCommand(() -> {ClimbingSubsystem.getInstance().open();})
        ).toggleOnFalse(
                new InstantCommand(() -> {ClimbingSubsystem.getInstance().close();})
        );

        // INTAKE
        driver.square().toggleOnTrue(
                Commands.sequence(
                        new InstantCommand(() -> {
                            IntakeSubsystem.getInstance().open();
                            IntakeSubsystem.getInstance().roll();
                            Feeder.getInstance().feed();
                        }),
                        Commands.waitUntil(() -> SimpleShooterSubsystem.getInstance().inShooter()),
                        new InstantCommand(() -> {Feeder.getInstance().stopFeeding();})
                )

        ).toggleOnFalse(
                new InstantCommand(() -> {
                    IntakeSubsystem.getInstance().close();
                    IntakeSubsystem.getInstance().stopRolling();
                })
        );

        // SHOOTER
        driver.circle().toggleOnTrue(
                new InstantCommand(() -> {
                    SimpleShooterSubsystem.getInstance().shoot();
                })
        ).toggleOnFalse(
                new InstantCommand(() -> {
                    SimpleShooterSubsystem.getInstance().stopShooting();
                })
        );

        // AMP SCORER
        driver.triangle().onTrue(
                Commands.sequence(
                        new InstantCommand(() -> {
                            AmpScorer.getInstance().openAmp();
                        }),
                        Commands.waitSeconds(0.5),
                        new InstantCommand(() -> {
                            AmpScorer.getInstance().rollAmp();
                        })
                )

        ).onFalse(
                new InstantCommand(() -> {
                    AmpScorer.getInstance().stopRollAmp();
                    AmpScorer.getInstance().closeAmp();
                })
        );

    }

    public static List<PathPlannerPath> splitting_paths_into_segments(PathPlannerPath path_from_file) {

        // Splitting path into segments:
        List<List<PathPoint>> segments_list = new ArrayList<>();
        segments_list.add(new ArrayList<PathPoint>());
        int segment_index = 0;

        for (PathPoint pathPoint : path_from_file.getAllPathPoints()) {
            segments_list.get(segment_index).add(pathPoint);

            if (pathPoint.rotationTarget != null) {

                segment_index += 1;

                segments_list.add(new ArrayList<PathPoint>());
                segments_list.get(segment_index).add(pathPoint);
            }
        }

        double totalTime = 0;


        ArrayList<PathPlannerTrajectory> pathPlannerTrajectories = new ArrayList<PathPlannerTrajectory>();
        ArrayList<PathPlannerPath> pathList = new ArrayList<PathPlannerPath>();

        for (List<PathPoint> segment : segments_list) {
            if (segment.size() <= 1) {
                continue;
            }
            PathPlannerPath path = PathPlannerPath.fromPathPoints(
                    segment,
                    segment.get(segment.size() / 2).constraints,
                    new GoalEndState(0, segment.get(segment.size() - 1).rotationTarget.getTarget()));

            pathList.add(path);

            PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(), segment.get(segment.size() - 1).rotationTarget.getTarget());

            pathPlannerTrajectories.add(trajectory);

            totalTime += trajectory.getTotalTimeSeconds();
        }

        return pathList;
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autosGenerator.getChosenCommand();
    }
}
