package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

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
//        autonomousSendableChooser.addOption("2020 auto 1", _2020_auto1());
//        autonomousSendableChooser.addOption("2020 shani's auto", _2020_auto2());
//        autonomousSendableChooser.addOption("2020 Auto 3", _2020_auto3());
//        autonomousSendableChooser.addOption("2020 Auto 4", _2020_auto4());

        autonomousSendableChooser.addOption("2024 auto 1", _2024_auto1());
        autonomousSendableChooser.addOption("2024 auto 2", _2024_auto1_with_markers());

        autonomousSendableChooser.addOption("2024 auto close shots", _2024_close_shots_auto());
//        autonomousSendableChooser.addOption("2024 shani's auto1", _2024_shani_auto1());
//        autonomousSendableChooser.addOption("2024 shani's auto 2", _2024_shani_auto2());
//        autonomousSendableChooser.addOption("shani auto 2 with markers",_2024_auto1_with_markers_shani());
//        autonomousSendableChooser.addOption("shani suto 5 game piece",_2024_auto_5_game_piece());
        autonomousSendableChooser.addOption("kyle1", kyle1());
        autonomousSendableChooser.addOption("Kyle 5 game piece", _5_piece_auto());

        SmartDashboard.putData(
                "reset pos", new InstantCommand(() -> {
                    NewPoseEstimatorSubsystem.getInstance().setCurrentPose(
                            new Pose2d(15.05, 5.50, Rotation2d.fromDegrees(0))
                    );
                })
        );

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

    public Command _2024_auto1() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("first auto 2024");
        List<PathPlannerPath> segments = RobotContainer.splitting_paths_into_segments(path_from_file);

        Command auto = new InstantCommand(
                () -> {}).
                andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingWithEvents(segments.get(0)),
                        new InstantCommand(IntakeSubsystem.getInstance()::open, IntakeSubsystem.getInstance())
                ));

        auto = auto.andThen(
                Commands.waitSeconds(3)
        );

        auto = auto.andThen(
                Commands.parallel(
                        new InstantCommand(IntakeSubsystem.getInstance()::close, IntakeSubsystem.getInstance()),
                        SimpleShooterSubsystem.getInstance().getDefaultShootingCommand()
                )
        );

        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingWithEvents(segments.get(1)),
                        Commands.sequence(
                                new WaitCommand(1),
                                new InstantCommand(IntakeSubsystem.getInstance()::open, IntakeSubsystem.getInstance())
                        )
                )
        );

        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingWithEvents(segments.get(2)),

                        Commands.sequence(
                                new WaitCommand(1),
                                new InstantCommand(IntakeSubsystem.getInstance()::close)
                        )
                )
        );

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingWithEvents(segments.get(3)),
                        new WaitCommand(1).andThen(new InstantCommand(IntakeSubsystem.getInstance()::open))
                )
        );

        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingWithEvents(segments.get(4)),
                        new WaitCommand(0.5).andThen(new InstantCommand(IntakeSubsystem.getInstance()::close))
                )
        );

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        return auto;
    }

    public Command _2024_auto1_with_markers() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("first auto 2024");

        return newSwerve.getDefaultPathFollowingCommand(path_from_file, poseEstimatorSubsystem);
    }

    public Command _2024_close_shots_auto() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Close autos 2024");
        List<PathPlannerPath> segments = RobotContainer.splitting_paths_into_segments(path_from_file);

        Command auto = new InstantCommand(
                () -> {

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

        auto = auto.andThen(
                newSwerve.getDefaultPathFollowingCommand(segments.get(2), poseEstimatorSubsystem)
        );







        return auto;
    }

    public Command _2024_shani_auto1() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("2024 Shani's Auto 1");
        Command auto = new InstantCommand(
                () -> {
                    NewPoseEstimatorSubsystem.getInstance().setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
                }
        );


        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand()
        );

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto = auto.andThen(newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem));

        auto = auto.andThen(
                Commands.deadline(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem),
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

        auto = auto.andThen(
                Commands.deadline(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem),
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

    public Command _2024_shani_auto2() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("2024 Shani's Auto 2");
        Command auto = new InstantCommand(
                () -> {
                    // NewPoseEstimatorSubsystem.getInstance().setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());
                }
        );


        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand(
        ));

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);
        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem)),
                new InstantCommand(IntakeSubsystem.getInstance()::open, IntakeSubsystem.getInstance())

        );


        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem),
                        new WaitCommand(0.5).andThen(new InstantCommand(IntakeSubsystem.getInstance()::close))
                ));

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem),
                        new InstantCommand(IntakeSubsystem.getInstance()::open, IntakeSubsystem.getInstance())
                )
        );

        auto = auto.andThen(
                Commands.parallel(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem),
                        new InstantCommand(IntakeSubsystem.getInstance()::close, IntakeSubsystem.getInstance())
                )

        );
        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());





        return auto;
    }
    public Command _2024_auto1_with_markers_shani() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("2024 Shani's Auto 2");

        return newSwerve.getDefaultPathFollowingCommand(path_from_file, poseEstimatorSubsystem);
    }

    public Command _2024_auto_5_game_piece() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("2024 5 Game piece");

        return newSwerve.getDefaultPathFollowingCommand(path_from_file, poseEstimatorSubsystem);
    }

    public Command kyle1() {
        PathPlannerPath first_part = PathPlannerPath.fromPathFile("kyle-0.1");
        Command auto = new InstantCommand(() -> {NewPoseEstimatorSubsystem.getInstance().setCurrentPose(
                new Pose2d(15, 6, Rotation2d.fromDegrees(0))
        );});

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(first_part),
                        Commands.sequence(
                                new WaitCommand(0.8),
                                new InstantCommand(IntakeSubsystem.getInstance()::open, IntakeSubsystem.getInstance())
                        )
                )
        );

        auto = auto.andThen(Commands.sequence(
                        new InstantCommand(IntakeSubsystem.getInstance()::close, IntakeSubsystem.getInstance()),
                        SimpleShooterSubsystem.getInstance().getDefaultShootingCommand()
                )

        );

        PathPlannerPath second_part = PathPlannerPath.fromPathFile("kyle-0.2");

        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(second_part),
                        Commands.sequence(
                                new WaitCommand(1.7),
                                new InstantCommand(IntakeSubsystem.getInstance()::open, IntakeSubsystem.getInstance())
                        )

                )
        );
        PathPlannerPath third_part = PathPlannerPath.fromPathFile("kyle-0.3");

        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(third_part),
                        Commands.sequence(
                                new WaitCommand(0.9),
                                new InstantCommand(IntakeSubsystem.getInstance()::close)
                        )

                )
        );

        auto = auto.andThen(
                SimpleShooterSubsystem.getInstance().getDefaultShootingCommand()
        );

        PathPlannerPath fourth_part = PathPlannerPath.fromPathFile("kyle-0.4");
        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(fourth_part),
                        Commands.sequence(
                                new WaitCommand(1.6),
                                new InstantCommand(IntakeSubsystem.getInstance()::open,IntakeSubsystem.getInstance())
                        )

                )
        );

        PathPlannerPath fifth_part = PathPlannerPath.fromPathFile("kyle-0.5");
        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(fifth_part),
                        Commands.sequence(
                                new WaitCommand(0.9),
                                new InstantCommand(IntakeSubsystem.getInstance()::close,IntakeSubsystem.getInstance())
                        )

                )
        );

        auto = auto.andThen(
                SimpleShooterSubsystem.getInstance().getDefaultShootingCommand()
        );

        return auto;
    }

    public Command _5_piece_auto() {
        PathPlannerPath first_part = PathPlannerPath.fromPathFile("5 piece auto-0.1");
        Command auto = new InstantCommand(() -> {NewPoseEstimatorSubsystem.getInstance().setCurrentPose(
                new Pose2d(15, 6, Rotation2d.fromDegrees(0))
        );});

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(first_part),
                        Commands.sequence(
                                new WaitCommand(0.7),
                                new InstantCommand(IntakeSubsystem.getInstance()::open, IntakeSubsystem.getInstance())
                        )
                )
        );

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        PathPlannerPath second_part = PathPlannerPath.fromPathFile("5 piece auto-0.2");
        auto = auto.andThen(
                NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(second_part)
        );

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        PathPlannerPath third_part = PathPlannerPath.fromPathFile("5 piece auto-0.3");

        auto = auto.andThen(
                NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(third_part)
        );

        PathPlannerPath fourth_part = PathPlannerPath.fromPathFile("5 piece auto-0.4");

        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(fourth_part),
                        Commands.sequence(
                                new WaitCommand(0.5),
                                new InstantCommand(IntakeSubsystem.getInstance()::close,IntakeSubsystem.getInstance())
                        )
                )

        );
        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        PathPlannerPath fifth_part = PathPlannerPath.fromPathFile("5 piece auto-0.5");

        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(fifth_part),
                        Commands.sequence(
                                new WaitCommand(2.35),
                                new InstantCommand(IntakeSubsystem.getInstance()::open,IntakeSubsystem.getInstance())
                        )
                )

        );

        PathPlannerPath sixth_part = PathPlannerPath.fromPathFile("5 piece auto-0.6");

        auto = auto.andThen(
                Commands.parallel(
                        NewSwerveDriveSubsystem.getInstance().getDefaultPathFollowingCommand(sixth_part),
                        Commands.sequence(
                                new WaitCommand(0.8),
                                new InstantCommand(IntakeSubsystem.getInstance()::close,IntakeSubsystem.getInstance())
                        )
                )

        );

        auto = auto.andThen(SimpleShooterSubsystem.getInstance().getDefaultShootingCommand());

        return auto;
    }



}