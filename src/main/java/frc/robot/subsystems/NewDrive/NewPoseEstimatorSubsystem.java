package frc.robot.subsystems.NewDrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
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

    SwerveModulePosition[] currentModulesPositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

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
        NewSwerveDriveSubsystem.odometryLock.lock();
        NewSwerveDriveSubsystem.getInstance().updateOdometryInputs();
        NewSwerveDriveSubsystem.odometryLock.unlock();
        // Update by DriveTrain:
        int delta_count = NewSwerveDriveSubsystem.getInstance().swerveModules[0].getDrivePositionArray().length;
        delta_count = Math.min(delta_count, NewSwerveDriveSubsystem.getInstance().gyroInformation.odometryYawPositions.length);

        for (int delta_index = 0; delta_index < delta_count; delta_index++) {
            for (int module_index = 0; module_index < 4; module_index++) {

                currentModulesPositions[module_index].distanceMeters = NewSwerveDriveSubsystem.getInstance().
                        swerveModules[module_index].getDrivePositionArray()[delta_index];

                currentModulesPositions[module_index].angle = NewSwerveDriveSubsystem.getInstance().
                        swerveModules[module_index].getTurnPositions()[delta_index];
            }

            poseEstimator.update(
                    NewSwerveDriveSubsystem.getInstance().gyroInformation.odometryYawPositions[delta_index],
                    currentModulesPositions);
        }


//         poseEstimator.update(Rotation2d.fromDegrees(drive.getYawDegrees()), drive.getModulesPosition());

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
        if (Robot.isReal())
            drive.pigeon2.setYaw(newPose.getRotation().getDegrees());

        System.out.println("signal based value: " + NewSwerveDriveSubsystem.getInstance().gyroInformation.yawSignal.getValueAsDouble());
        System.out.println("our based value: " + NewSwerveDriveSubsystem.getInstance().getYawDegrees());

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