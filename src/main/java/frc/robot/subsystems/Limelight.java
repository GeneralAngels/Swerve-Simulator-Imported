package frc.robot.subsystems;


import java.util.Arrays;
import edu.wpi.first.hal.AllianceStationID;
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


import java.util.Arrays;
import java.util.function.Supplier;

public class Limelight extends SubsystemBase {

    private static Limelight instace = null;
    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static final Supplier<Double> currAngle = NewSwerveDriveSubsystem.getInstance()::getYawDegrees;

    public static double[] visionRet = new double[7];

    static double[] empty = new double[7];
    static double[] empty_1 = new double[1];

    public static Limelight getInstace(){
        if (instace == null){
            instace = new Limelight();
        }
        return  instace;
    }

    public static LimelightMeasurement MegaTagEstimate() {

        // Get the number of detected AprilTags
        int numTags = limelight.getEntry("tcornxy").getDoubleArray(empty_1).length / 4;
        if (numTags == 0) return null;

        if (get_alliance() == DriverStation.Alliance.Blue){
            visionRet = limelight.getEntry("botpose_wpiblue").getDoubleArray(empty);
        }
        else if (get_alliance() == DriverStation.Alliance.Red){
            visionRet = limelight.getEntry("botpose_wpired").getDoubleArray(empty);
        }

        Pose3d robotPose = new Pose3d(visionRet[0], visionRet[1], visionRet[2],
                new Rotation3d(visionRet[3], visionRet[4], visionRet[5]));


        double estimatedRotation = robotPose.getRotation().getZ();

        // Check if the estimated rotation lines up with the current gyro value
        if (Math.abs(currAngle.get() - estimatedRotation) > Constants.PoseEstimatorConstants.maxEstimatedAngleError && numTags == 1){
            return null;
        }


        double timestamp = Timer.getFPGATimestamp() - (visionRet[6] / 1000.0);
        var fieldPose = new Pose2d(robotPose.getX(),
                robotPose.getY(),
                new Rotation2d(robotPose.getRotation().getZ()));

        return new LimelightMeasurement(fieldPose, timestamp);
    }

    public static DriverStation.Alliance get_alliance() {
        var optional_ds_alliance = DriverStation.getAlliance();

        if (optional_ds_alliance.isPresent()) {
            return optional_ds_alliance.get();
        }
        else {
//            System.out.println("[WARNING] DRIVER STATION ALLIANCE WAS NOT PICKED YET");
            var raw_station = DriverStation.getRawAllianceStation();

            if (raw_station == AllianceStationID.Red1 || raw_station == AllianceStationID.Red2 || raw_station == AllianceStationID.Red3) {
                return DriverStation.Alliance.Red;
            }
            else {
                return DriverStation.Alliance.Blue;
            }
        }
    }
}

