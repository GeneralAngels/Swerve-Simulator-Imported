package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;
import frc.robot.subsystems.Shooter;

import java.util.List;

public class AutosGenerator {
    NewPoseEstimatorSubsystem poseEstimatorSubsystem = NewPoseEstimatorSubsystem.getInstance();
    NewSwerveDriveSubsystem newSwerve = NewSwerveDriveSubsystem.getInstance();

    SendableChooser<Command> autonomousSendableChooser;

    public AutosGenerator() {
        autonomousSendableChooser = new SendableChooser<Command>();
        autonomousSendableChooser.setDefaultOption("2023 autonomous example path", get2023_example_auto());
        autonomousSendableChooser.addOption("2020 auto 1", _2020_auto1());
//        autonomousSendableChooser.addOption("2020 shani's auto", _2020_auto2());
//        autonomousSendableChooser.addOption("2020 Auto 3", _2020_auto3());


        SmartDashboard.putData("Autonomous chooser", autonomousSendableChooser);
    }

    public Command getChosenCommand() {
        return autonomousSendableChooser.getSelected();
    }

    public Command get2023_example_auto() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Example 2023 path");

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);

        var auto_command = new InstantCommand(() -> {
            poseEstimatorSubsystem.setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
        }).andThen();

        var path_planner_command = newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem)
                .andThen(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem)
                ).andThen(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem)
                ).andThen(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem)
                );

        return auto_command.andThen(path_planner_command);
    }

    public Command _2020_auto1() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto1");
        var auto_command = Commands.sequence();

        auto_command = auto_command.andThen(
                new InstantCommand(() -> {
                    Shooter.getInstance().setDesiredVelocity(300); // in RPM
                }).andThen(
                        new WaitCommand(2)
                ).andThen(
                        new InstantCommand(() -> {
                            System.out.println("Finished shooting");
                            Shooter.getInstance().setDesiredVelocity(0);
                        })
                )
        );

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));

        auto_command = auto_command.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem),
                        new InstantCommand(() -> {
                            System.out.println("COLLECTING");
                        })
                )
        );

        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem));

        return auto_command;
    }

    /*
    public Command _2020_auto2() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto2");
        var auto_command = new InstantCommand(
                () -> {
                    poseEstimatorSubsystem.setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
                }).andThen();

        auto_command = auto_command.andThen(
                new InstantCommand(() -> {
                    Shooter.getInstance().setDesiredVelocity(300); // in RPM
                }).andThen(
                        new WaitCommand(2)
                ).andThen(
                        new InstantCommand(() -> {
                            System.out.println("Finished shooting");
                            Shooter.getInstance().setDesiredVelocity(0);
                        })
                )
        );

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));

        auto_command = auto_command.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem),
                        new InstantCommand(() -> {
                            System.out.println("COLLECTING 1");
                        })
                )
        );

        auto_command = auto_command.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem),
                        new InstantCommand(() -> {
                            System.out.println("COLLECTING 2");
                        })
                )
        );


        auto_command = auto_command.andThen(
                newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem) , newSwerve.getDefaultPathFollowingCommand(paths.get(4), poseEstimatorSubsystem));

        return auto_command;
    }

    public Command _2020_auto3() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto3");
        var auto_command = new InstantCommand(
                () -> {
                    poseEstimatorSubsystem.setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
                }).andThen();



        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));

        auto_command = auto_command.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem),
                        new InstantCommand(() -> {
                            System.out.println("COLLECTING 1");
                        })
                )
        );

        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem));


        auto_command = auto_command.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem),
                        new InstantCommand(() -> {
                            System.out.println("COLLECTING 2");
                        })
                )
        );

        auto_command = auto_command.andThen(
                newSwerve.getDefaultPathFollowingCommand(paths.get(4), poseEstimatorSubsystem),
                new InstantCommand(() -> {
                    Shooter.getInstance().setDesiredVelocity(300); // in RPM
                }).andThen(
                        new WaitCommand(2)
                ).andThen(
                        new InstantCommand(() -> {
                            System.out.println("Finished shooting");
                            Shooter.getInstance().setDesiredVelocity(0);
                        })
                )
        );




        return auto_command;
    }
    */
}
