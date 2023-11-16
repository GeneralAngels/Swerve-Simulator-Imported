package frc.robot.subsystems.NewDrive;

import java.sql.Driver;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewPoseEstimatorSubsystem extends SubsystemBase {

    private PhotonCamera photonCamera;
    private NewSwerveDriveSubsystem drive;

    // Physical location of the camera on the robot, relative to the center of the
    // robot.
    private static final Transform2d CAMERA_TO_ROBOT =
            new Transform2d(new Translation2d(Units.inchesToMeters(12.75), 0.0), new Rotation2d(0.0));

    // Ordered list of target poses by ID (WPILib is adding some functionality for
    // this)
    private static final List<Pose2d> targetPoses = Collections.unmodifiableList(List.of(
            new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(39.4375), Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(84), 0.0, Rotation2d.fromDegrees(180))));

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    private Pose3d robotPose;
    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private double[] visionRet = new double[7];
    private boolean savedJson = false;

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.
    private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
    private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01));
    private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
    private SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();

    public NewPoseEstimatorSubsystem(PhotonCamera photonCamera, NewSwerveDriveSubsystem drivetrainSubsystem) {
        this.photonCamera = photonCamera;
        this.drive = drivetrainSubsystem;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        poseEstimator = new SwerveDrivePoseEstimator(
                drive.kinematics,
                Rotation2d.fromDegrees(drive.getYawDegrees()),
                drive.getModulesPosition(),
                new Pose2d(),
                stateStdDevs, visionMeasurementStdDevs
        );


        tab.addString("Pose (X, Y)", this::getFomattedPose).withPosition(0, 4);
        tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
        tab.add(field2d);
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

    @Override
    public void periodic() {
        // Update pose estimator with visible targets
//        System.out.println(get_alliance());
        if (get_alliance() == DriverStation.Alliance.Blue){
            visionRet = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        }
        else if (get_alliance() == DriverStation.Alliance.Red){
            visionRet = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        }

        robotPose = new Pose3d(visionRet[0], visionRet[1], visionRet[2],
                new Rotation3d(visionRet[3], visionRet[4], visionRet[5]));

//        System.out.println(robotPose);

        if (!savedJson) {
//            System.out.println(limelight.getEntry("json").toString());
            savedJson = true;
        }

        // Update by DriveTrain:
        poseEstimator.update(Rotation2d.fromDegrees(drive.getYawDegrees()), drive.getModulesPosition());

        var current_pose = getCurrentPose();
        SmartDashboard.putNumber("pose.x", current_pose.getX());
        SmartDashboard.putNumber("pose.y", current_pose.getY());
        SmartDashboard.putNumber("pose.angle", current_pose.getRotation().getDegrees());

        Logger.recordOutput("MyPose", current_pose);


        field2d.setRobotPose(getCurrentPose());
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f)",
                Units.metersToInches(pose.getX()),
                Units.metersToInches(pose.getY()));
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        if (Robot.isSimulation()) {
            drive.pigeonSimCollection.setRawHeading(0);
        }

        drive.pigeon2.setYaw(newPose.getRotation().getDegrees());
        poseEstimator.resetPosition(Rotation2d.fromDegrees(drive.getYawDegrees()), drive.getModulesPosition(), newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

}