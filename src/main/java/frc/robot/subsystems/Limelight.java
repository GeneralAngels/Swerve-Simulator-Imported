package frc.robot.subsystems;


import java.util.Arrays;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;
import frc.robot.Utils.LimelightMeasurement;


import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {

    private static Limelight instace = null;
    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static final Supplier<Double> currAngle = NewSwerveDriveSubsystem.getInstance()::getYawDegrees;

    public static double[] visionRet = new double[7];
    private static double[] tagPose = new double[7];
    public static Pose2d fieldPose;

    static double[] empty = new double[7];
    static double[] empty_1 = new double[1];

    static Translation2d target_pose_camerspace = new Translation2d();

    public static Limelight getInstace() {
        if (instace == null) {
            instace = new Limelight();
        }
        return instace;
    }

    public static double getDistanceToTarget() {
        var cameraspace_limelight_result = limelight.getEntry("targetpose_cameraspace").getDoubleArray(empty);
        return Math.hypot(cameraspace_limelight_result[0], cameraspace_limelight_result[1]);
    }

    public static LimelightMeasurement MegaTagEstimate() {

        // Get the number of detected AprilTags
        int numTags = limelight.getEntry("tcornxy").getDoubleArray(empty_1).length / 4;
        if (numTags == 0) return null;

        visionRet = limelight.getEntry("botpose_wpiblue").getDoubleArray(empty);
        tagPose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(empty);


        fieldPose = new Pose2d(visionRet[0],
                visionRet[1],
                Rotation2d.fromDegrees(visionRet[5]));

        Logger.recordOutput("LimelightRawPose", fieldPose);

        Logger.recordOutput("distance", Math.hypot(fieldPose.getX(), fieldPose.getY()));

        double estimatedRotation = fieldPose.getRotation().getDegrees();

        // Check if the estimated rotation lines up with the current gyro value
        if (Math.abs(currAngle.get() - estimatedRotation) > Constants.PoseEstimatorConstants.maxEstimatedAngleError &&
                numTags == 1 ||
                tagPose[2] > 3.5) {
            Logger.recordOutput("Using Tag", false);
            return null;
        }
        Logger.recordOutput("Using Tag", true);


        double timestamp = Timer.getFPGATimestamp() - (visionRet[6] / 1000.0);
        return new LimelightMeasurement(fieldPose, timestamp);
    }

    public static DriverStation.Alliance get_alliance() {
        var optional_ds_alliance = DriverStation.getAlliance();

        if (optional_ds_alliance.isPresent()) {
            return optional_ds_alliance.get();
        } else {
//            System.out.println("[WARNING] DRIVER STATION ALLIANCE WAS NOT PICKED YET");
            var raw_station = DriverStation.getRawAllianceStation();

            if (raw_station == AllianceStationID.Red1 || raw_station == AllianceStationID.Red2 || raw_station == AllianceStationID.Red3) {
                return DriverStation.Alliance.Red;
            } else {
                return DriverStation.Alliance.Blue;
            }
        }
    }
}

