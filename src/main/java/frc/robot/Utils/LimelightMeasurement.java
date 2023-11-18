package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;

public class LimelightMeasurement {

    public Pose2d pose;
    public double timestamp;

    public LimelightMeasurement(Pose2d pose, double timestamp){
        this.pose = pose;
        this.timestamp = timestamp;
    }
}