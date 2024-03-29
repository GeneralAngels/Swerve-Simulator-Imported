package frc.robot.subsystems.NewDrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight;
import frc.robot.Utils.LimelightMeasurement;
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;
import org.littletonrobotics.junction.Logger;

public class NewPoseEstimatorSubsystem extends TimeMeasurementSubsystem {
    private final NewSwerveDriveSubsystem drive = NewSwerveDriveSubsystem.getInstance();
    private static NewPoseEstimatorSubsystem instance = null;

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.
    private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
    private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01));
    private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(25));
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    public static NewPoseEstimatorSubsystem getInstance() {
        if (instance == null) {
            instance = new NewPoseEstimatorSubsystem();
        }
        return instance;
    }

    public NewPoseEstimatorSubsystem() {
        poseEstimator = new SwerveDrivePoseEstimator(
                drive.kinematics,
                Rotation2d.fromDegrees(drive.getYawDegrees()),
                drive.getModulesPosition(),
                new Pose2d(),
                stateStdDevs, visionMeasurementStdDevs
        );

        SmartDashboard.putData("field", field2d);
        setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void _periodic() {
        // Update by DriveTrain:
        poseEstimator.update(Rotation2d.fromDegrees(drive.getYawDegrees()), drive.getModulesPosition());

        var current_pose = getCurrentPose();
        Logger.recordOutput("PoseEstimatorPose", current_pose);

        field2d.setRobotPose(getCurrentPose());

        // Update pose estimator with visible targets
        LimelightMeasurement limelightMeasurement = Limelight.MegaTagEstimate();
        if (limelightMeasurement == null) {
            Logger.recordOutput("Tag in sight", false);
            return;
        }

        Logger.recordOutput("Tag in sight", true);
        poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestamp);
    }

    private String getFormattedPose() {
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
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        drive.pigeon2.setYaw(newPose.getRotation().getDegrees());
        System.out.println("Resetting position");
        poseEstimator.resetPosition(Rotation2d.fromDegrees(drive.getYawDegrees()), drive.getModulesPosition(), newPose);
        System.out.println(getCurrentPose().getX() + " " + getCurrentPose().getY());
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

}