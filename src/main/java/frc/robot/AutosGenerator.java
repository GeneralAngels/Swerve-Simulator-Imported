package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

import java.util.List;

public class AutosGenerator {
    NewPoseEstimatorSubsystem poseEstimatorSubsystem = NewPoseEstimatorSubsystem.getInstance();
    NewSwerveDriveSubsystem newSwerve = NewSwerveDriveSubsystem.getInstance();

    public AutosGenerator() {

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
