package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;
import frc.robot.subsystems.Shooter;

import java.util.List;

public class AutosGenerator {
    NewPoseEstimatorSubsystem poseEstimatorSubsystem = NewPoseEstimatorSubsystem.getInstance();
    NewSwerveDriveSubsystem newSwerve = NewSwerveDriveSubsystem.getInstance();

    SendableChooser<Command> autonomousSendableChooser;

    public AutosGenerator() {
        NamedCommands.registerCommand(
                "intake",
                new InstantCommand(() -> {
                    System.out.println("opening");
                    IntakeSubsystem.getInstance().open();
                }));

        autonomousSendableChooser = new SendableChooser<Command>();
        autonomousSendableChooser.setDefaultOption("2023 autonomous example path", get2023_example_auto());
        autonomousSendableChooser.addOption("2020 auto 1", _2020_auto1());
        autonomousSendableChooser.addOption("2020 shani's auto", _2020_auto2());
        autonomousSendableChooser.addOption("2020 Auto 3", _2020_auto3());
        autonomousSendableChooser.addOption("2020 Auto 4", _2020_auto4());

        autonomousSendableChooser.addOption("2024 auto 1", _2024_auto1());
        autonomousSendableChooser.addOption("2024 auto 2", _2024_auto1_with_markers());

        autonomousSendableChooser.addOption("2024 auto close shots", _2024_close_shots_auto());

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

        var path_planner_command = newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem).andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem)).andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem)).andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem));

        return auto_command.andThen(path_planner_command);
    }

    public Command _2020_auto1() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto1");
        var auto_command = Commands.sequence();

        auto_command = auto_command.andThen(new InstantCommand(() -> {
            Shooter.getInstance().setDesiredVelocity(300); // in RPM
        }).andThen(new WaitCommand(2)).andThen(new InstantCommand(() -> {
            System.out.println("Finished shooting");
            Shooter.getInstance().setDesiredVelocity(0);
        })));

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));

        auto_command = auto_command.andThen(Commands.parallel(newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem), new InstantCommand(() -> {
            System.out.println("COLLECTING");
        })));

        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem));

        return auto_command;
    }

    public Command _2020_auto2() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto2");
        var auto_command = new InstantCommand(() -> {
            poseEstimatorSubsystem.setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
        }).andThen();

        auto_command = auto_command.andThen(new InstantCommand(() -> {
            Shooter.getInstance().setDesiredVelocity(300); // in RPM
        }).andThen(new WaitCommand(2)).andThen(new InstantCommand(() -> {
            System.out.println("Finished shooting");
            Shooter.getInstance().setDesiredVelocity(0);
        })));

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));

        auto_command = auto_command.andThen(Commands.parallel(newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem), new InstantCommand(() -> {
            System.out.println("COLLECTING 1");
        })));

        auto_command = auto_command.andThen(Commands.parallel(newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem), new InstantCommand(() -> {
            System.out.println("COLLECTING 2");
        })));


        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem), newSwerve.getDefaultPathFollowingCommand(paths.get(4), poseEstimatorSubsystem));

        return auto_command;
    }

    public Command _2020_auto3() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto3");
        var auto_command = new InstantCommand(() -> {
            poseEstimatorSubsystem.setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
        }).andThen();


        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));

        auto_command = auto_command.andThen(Commands.parallel(newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem), new InstantCommand(() -> {
            System.out.println("COLLECTING 1");
        })));

        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem));


        auto_command = auto_command.andThen(Commands.parallel(newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem), new InstantCommand(() -> {
            System.out.println("COLLECTING 2");
        })));

        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(4), poseEstimatorSubsystem), new InstantCommand(() -> {
            Shooter.getInstance().setDesiredVelocity(300); // in RPM
        }).andThen(new WaitCommand(2)).andThen(new InstantCommand(() -> {
            System.out.println("Finished shooting");
            Shooter.getInstance().setDesiredVelocity(0);
        })));


        return auto_command;
    }

    public Command _2020_auto4() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto4.2");
        var auto_command = Commands.sequence();

        auto_command = auto_command.andThen(new InstantCommand(() -> {
            Shooter.getInstance().setDesiredVelocity(300); // in RPM
        }).andThen(new WaitCommand(2)).andThen(new InstantCommand(() -> {
            System.out.println("Finished shooting");
            Shooter.getInstance().setDesiredVelocity(0);
        })));

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));


        auto_command = auto_command.andThen(Commands.parallel(newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem), new InstantCommand(() -> {
            System.out.println("COLLECTING");
        })));

        auto_command = auto_command.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem));


        return auto_command;
    }

    public Command _2024_auto1() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("first auto 2024");
        List<PathPlannerPath> segments = RobotContainer.splitting_paths_into_segments(path_from_file);

        System.out.println("size " + segments.get(0).getEventMarkers().size());

        Command auto = new InstantCommand(
                () -> {
                    NewPoseEstimatorSubsystem.getInstance().setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
                    IntakeSubsystem.getInstance().open();
                })
                .andThen(newSwerve.getDefaultPathFollowingWithEvents(segments.get(0)));

        auto = auto.andThen(new InstantCommand(() -> {
            System.out.println("shooting");
            IntakeSubsystem.getInstance().close();
        }));

        auto = auto.andThen(newSwerve.getDefaultPathFollowingWithEvents(segments.get(1)));

        auto = auto.andThen(newSwerve.getDefaultPathFollowingWithEvents(segments.get(2)));

        auto = auto.andThen(new InstantCommand(() -> {
            System.out.println("shooting");
        }));

        auto = auto.andThen(newSwerve.getDefaultPathFollowingWithEvents(segments.get(3)));

        auto = auto.andThen(newSwerve.getDefaultPathFollowingWithEvents(segments.get(4)));

        return auto;
    }

    public Command _2024_auto1_with_markers() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("first auto 2024");

        return newSwerve.getDefaultPathFollowingWithEvents(path_from_file);
    }

    public Command _2024_close_shots_auto() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Close autos 2024");
        List<PathPlannerPath> segments = RobotContainer.splitting_paths_into_segments(path_from_file);

        Command auto = new InstantCommand(
                () -> {
                    NewPoseEstimatorSubsystem.getInstance().setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
                }
        );

        auto = auto.andThen(new InstantCommand(
                () -> {
                    System.out.println("shooting");
                }
        ));

        auto = auto.andThen(
                newSwerve.getDefaultPathFollowingCommand(segments.get(0), poseEstimatorSubsystem)
        );

        auto = auto.andThen(
                Commands.deadline(
                        newSwerve.getDefaultPathFollowingCommand(segments.get(1), poseEstimatorSubsystem),
                        Commands.repeatingSequence(
                                new InstantCommand(
                                        () -> {
                                            System.out.println("collecting");
                                        }
                                ),

                                new WaitCommand(0.4),

                                new InstantCommand(
                                        () -> System.out.println("shooting")
                                )
                        )
                )
        );


        return auto;
    }
}
