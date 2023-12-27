import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.path.PathPlannerPath;

public class new_pathplanner_test {
    public new_pathplanner_test() {}

    @Test
    public void test() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Testing");

        List<PathPlannerPath> _list = RobotContainer.paths_splitted_by_EventMarks(path_from_file);

        System.out.println("list length: " + _list.size());
    }
}
