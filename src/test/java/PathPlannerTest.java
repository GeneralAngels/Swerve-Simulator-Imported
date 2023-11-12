import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

public class PathPlannerTest {
    public List<PathPlannerPath> splitting_paths_into_segments_2(PathPlannerPath path_from_file) {
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

    public List<PathPlannerTrajectory> splitting_paths_into_segments(PathPlannerPath path_from_file) {
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
                segments_list.add(new ArrayList<PathPoint>());
            }
        }

        double totalTime = 0;

        System.out.println(segments_list.toString());

        ArrayList<PathPlannerTrajectory> pathPlannerTrajectories = new ArrayList<PathPlannerTrajectory>();

        for (List<PathPoint> segment : segments_list) {
            if (segment.isEmpty()) {
                break;
            }
            PathPlannerPath path = PathPlannerPath.fromPathPoints(
                    segment,
                    path_from_file.getGlobalConstraints(),
                    new GoalEndState(0, segment.get(segment.size() - 1).holonomicRotation));

            PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds());

            pathPlannerTrajectories.add(trajectory);

            System.out.println(trajectory.getTotalTimeSeconds());
            totalTime += trajectory.getTotalTimeSeconds();
        }

        return pathPlannerTrajectories;
    }

    @Test
    public void testing_paths() {
        PathPlannerPath path_from_file = PathPlannerPath.fromPathFile("Example Path");
        PathPlannerTrajectory og_trajectory = new PathPlannerTrajectory(path_from_file, new ChassisSpeeds());

        List<PathPlannerPath> paths = splitting_paths_into_segments_2(path_from_file);
        double totalTime = 0;

        for (PathPlannerPath path : paths) {
            PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds());
            totalTime += trajectory.getTotalTimeSeconds();
        }

        System.out.println("original: " + og_trajectory.getTotalTimeSeconds());
        System.out.println("total time: " + totalTime);
    }
}
