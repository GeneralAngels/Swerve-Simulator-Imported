package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

import java.util.List;

public class AutosGenerator {
    NewPoseEstimatorSubsystem poseEstimatorSubsystem;
    NewSwerveDriveSubsystem newSwerve;

    SendableChooser<Command> autoChooser;

    public AutosGenerator(NewPoseEstimatorSubsystem poseEstimatorSubsystem, NewSwerveDriveSubsystem newSwerve) {
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.newSwerve = newSwerve;

        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Default 2023 example auto", get2023_example_auto());
        autoChooser.addOption("2020 auto 1", _2020_auto1());

        SmartDashboard.putData("auto chooser", autoChooser);
    }

    public Command getAutoChosen() {
        return autoChooser.getSelected();
    }

    public Command get2023_example_auto() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Example 2023 path");

        List<PathPlannerPath> paths = RobotContainer.splitting_paths_into_segments(path_from_file);

        poseEstimatorSubsystem.setCurrentPose(path_from_file.getPreviewStartingHolonomicPose());

        if (Robot.isSimulation()) {
            newSwerve.pigeonSimCollection.setRawHeading(0);
        }
        newSwerve.pigeon2.setYaw(0);

        var path_planner_command = newSwerve.getDefaultPathFollowingCommand(paths.get(0), poseEstimatorSubsystem)
                .andThen(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(1), poseEstimatorSubsystem)
                ).andThen(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(2), poseEstimatorSubsystem)
                ).andThen(
                        newSwerve.getDefaultPathFollowingCommand(paths.get(3), poseEstimatorSubsystem)
                );

        return path_planner_command;
    }

    public Command _2020_auto1() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Auto1");
        var auto_command = Commands.sequence();

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
}
