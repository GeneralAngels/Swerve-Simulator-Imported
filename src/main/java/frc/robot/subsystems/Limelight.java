package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;
import frc.robot.Utils.LimelightMeasurement;


import java.util.function.Supplier;

public class Limelight extends SubsystemBase {

    private static Limelight instace = null;
    private static double[] visionRet = new double[7];
    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static Supplier<Double> currAngle = NewSwerveDriveSubsystem.getInstance()::getYawDegrees;

    public static Limelight getInstace(){
        if (instace == null){
            instace = new Limelight();
        }
        return  instace;
    }

    public static LimelightMeasurement MegaTagEstimate() {
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)){
            System.out.println("BLUE");
            visionRet = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        }
        else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
            visionRet = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        }

        Pose3d robotPose = new Pose3d(visionRet[0], visionRet[1], visionRet[2],
                new Rotation3d(visionRet[3], visionRet[4], visionRet[5]));

        // Get the number of detected AprilTags
        int numTags = limelight.getEntry("tcornxy").getDoubleArray(new double[1]).length / 4;
        double estimatedRotation = robotPose.getRotation().getZ();

        // Check if the estimated rotation lines up with the current gyro value
        if (Math.abs(currAngle.get()) - Math.abs(estimatedRotation) > Constants.PoseEstimatorConstants.maxEstimatedAngleError){
            return null;
        }


        double timestamp = Timer.getFPGATimestamp() - (visionRet[6] / 1000.0);
        var fieldPose = new Pose2d(robotPose.getX(),
                robotPose.getY(),
                new Rotation2d(robotPose.getRotation().getZ()));


        return new LimelightMeasurement(fieldPose, timestamp);
    }
}

