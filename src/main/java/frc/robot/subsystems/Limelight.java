package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.Arrays;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.Num;
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
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;


import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import static org.opencv.core.CvType.CV_64FC1;

public class Limelight extends SubsystemBase {

    private static Limelight instace = null;
    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static final Supplier<Double> currAngle = NewSwerveDriveSubsystem.getInstance()::getYawDegrees;

    private static Mat mCameraMatrix = new Mat(3, 3, CV_64FC1);
    private static Mat mDistortionCoeffients = new Mat(1, 5, CV_64FC1);
    public static double[] visionRet = new double[7];

    static double[] empty = new double[7];
    static double[] empty_1 = new double[1];
    static Number[] corners;
    static Translation2d corner;
    static Number[] empty_number = new Number[] {0,0,0,0,0};
    public static Limelight getInstace(){
        if (instace == null){
            instace = new Limelight();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    mCameraMatrix.put(i, j, Constants.VisionConstants.LIMELIGHT_CAMERA_MATRIX[i][j]);
                }
            }

            for (int i = 0; i < 5; i++){
                mDistortionCoeffients.put(0, i, Constants.VisionConstants.LIMELIGHT_CAMERA_MATRIX[i]);
            }
        }
        return  instace;
    }
    public static Translation2d ChezyEstimate() {
        corners = limelight.getEntry("tcornxy").getNumberArray(empty_number);

        for (int i = 0; i < corners.length; i++){
            corner = new Translation2d(corners[i].doubleValue(), corners[i + 1].doubleValue());
            double[] undistortedNormalizedPixelValues;
            try {
                undistortedNormalizedPixelValues = undistortFromOpenCV(new double[]{
                        corner.getX() / Constants.VisionConstants.CAMERA_WIDTH,
                        corner.getY() / Constants.VisionConstants.CAMERA_HEIGHT
                });

            } catch (Exception e) {
                DriverStation.reportError("Undistorting Point Throwing Error!", false);
                return null;
            }

            double y_pixels = undistortedNormalizedPixelValues[0];
            double z_pixels = undistortedNormalizedPixelValues[1];


            //Negate OpenCV Undistorted Pixel Values to Match Robot Frame of Reference
            //OpenCV: Positive Downward and Right
            //Robot: Positive Upward and Left
            double nY = -(y_pixels - mCameraMatrix.get(0, 2)[0]);// -(y_pixels * 2.0 - 1.0);
            double nZ = -(z_pixels - mCameraMatrix.get(1, 2)[0]);// -(z_pixels * 2.0 - 1.0);

            double y = nY / mCameraMatrix.get(0, 0)[0];
            double z = nZ / mCameraMatrix.get(1, 1)[0];

            Translation2d xz_plane_translation = new Translation2d(1.0, z).rotateBy(Rotation2d.fromDegrees(Constants.VisionConstants.CAMERA_TO_ROBOT.getRotation().getY()));
            double x = xz_plane_translation.getX();
            z = xz_plane_translation.getY();

            double offset =  isTopCorner ? Units.inches_to_meters(3) : - Units.inches_to_meters(3);
            // find intersection with the goal
            double differential_height = mTagMap.get(target.getTagId()).getHeight() - Constants.kLensHeight + offset;
            if ((z > 0.0) == (differential_height > 0.0)) {
                double scaling = differential_height / z;
                double distance = Math.hypot(x, y) * scaling;
                Rotation2d angle = new Rotation2d(x, y, true);
                return new Translation2d(distance * angle.cos(), distance * angle.sin());
            }
            return null;
        }
        }

    }
    public static LimelightMeasurement MegaTagEstimate() {

        // Get the number of detected AprilTags
        if (limelight.getEntry("tcornxy").getDoubleArray(empty).length / 4 == 0) return null;

        if (get_alliance() == DriverStation.Alliance.Blue){
            visionRet = limelight.getEntry("botpose_wpiblue").getDoubleArray(empty);
        }
        else if (get_alliance() == DriverStation.Alliance.Red){
            visionRet = limelight.getEntry("botpose_wpired").getDoubleArray(empty);
        }

        SmartDashboard.putBoolean("limelight messaurment empty", visionRet == empty);

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
    private static List<Translation2d> getCorners(Number[] tcornxy) {
        // Check if there is a non even number of corners
        if (tcornxy.length % 2 != 0) {
            return List.of();
        }

        ArrayList<Translation2d> corners = new ArrayList<>(tcornxy.length / 2);
        for (int i = 0; i < tcornxy.length; i += 2) {
            corners.add(new Translation2d(tcornxy[i].doubleValue(), tcornxy[i + 1].doubleValue()));
        }

        return corners;
    }
    private static synchronized double[] undistortFromOpenCV(double[] point) throws Exception {
        Point coord = new Point();
        coord.x = point[0];
        coord.y = point[1];

        MatOfPoint2f coordMat = new MatOfPoint2f(coord);

        Point dstCoord = new Point();
        MatOfPoint2f dst = new MatOfPoint2f(dstCoord);
        Calib3d.undistortImagePoints(coordMat,
                dst,
                mCameraMatrix,
                mDistortionCoeffients);

        return dst.get(0, 0);
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

