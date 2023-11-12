package frc.robot.subsystems;

import com.ctre.phoenix.time.StopWatch;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.wpiPoseEstimator;
import frc.robot.subsystems.Drive.SwerveConstants;
import frc.robot.subsystems.Drive.SwerveDriveTrain;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.sql.Driver;
import java.util.Collections;
import java.util.List;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;


public class PoseEstimator extends SubsystemBase {

    public final PhotonCamera photonCamera;
    private final SwerveDriveTrain drivetrainSubsystem;
    private static List<Pose3d> targetPoses = null;
    // Ordered list of target poses by ID (WPILib is adding some functionality for
    // this)


    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
     * and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.33, 0.33, Units.degreesToRadians(10));

    private final wpiPoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();

    private double previousPipelineTimestamp = 0;

    public Pose2d m_visionPose = new Pose2d();

    GenericEntry cameraConnected = Shuffleboard.getTab("PIT").add("limelight connected", false).getEntry();

    StopWatch stopWatch = new StopWatch();
    double lastVisionTime = 0;

    double counter = 0;
    double average = 0;

    public PoseEstimator(PhotonCamera photonCamera, SwerveDriveTrain drivetrainSubsystem) {
        this.photonCamera = photonCamera;
        this.drivetrainSubsystem = drivetrainSubsystem;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");

        poseEstimator = new wpiPoseEstimator(
                SwerveConstants.kinematics,
                Rotation2d.fromDegrees(-drivetrainSubsystem.gyro.getYaw()),
                drivetrainSubsystem.getModulePositions(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                stateStdDevs,
                visionMeasurementStdDevs
        );

        stopWatch.start();

        // tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

        initAlliance();

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
        //   System.out.println("Adjusting for RED.");
        //   targetPoses = Collections.unmodifiableList(
        //   List.of(
        //     // red part:
        //     new Pose3d(0, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(180.0))),
        //     new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),
        //     new Pose3d(0, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),

        // substations part:
        // new Pose3d(-0.66, 4.00177, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),
        // new Pose3d(-15.151354, 4.00177, 0, new Rotation3d(0, 0, degreesToRadians(0))),

        //     // blue part:
        //     new Pose3d(-14.486128, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(0))),
        //     new Pose3d(-14.486128, 0, 0, new Rotation3d(0, 0, degreesToRadians(0))),
        //     new Pose3d(-14.486128, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(0)))
        //   ));
        // }

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
        //   System.out.println("Adjusting for BLUE.");
        //   targetPoses = Collections.unmodifiableList(
        //   List.of(
        //     // red part:
        //     new Pose3d(14.486128, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(0))),
        //     new Pose3d(14.486128, 0, 0, new Rotation3d(0, 0, degreesToRadians(0))),
        //     new Pose3d(14.486128, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(0))),

        //     // substations part:
        //     new Pose3d(15.151354, 4.00177, 0, new Rotation3d(0, 0, degreesToRadians(0))),
        //     new Pose3d(-0.66, 4.00177, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),

        //     // blue part:
        //     new Pose3d(0, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(180.0))),
        //     new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),
        //     new Pose3d(0, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(180.0)))
        //   ));

        // }
    }

    public void initAlliance() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            System.out.println("Adjusting for RED.");
            targetPoses = Collections.unmodifiableList(
                    List.of(
                            // red part:
                            new Pose3d(0, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(180.0))),
                            new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),
                            new Pose3d(0, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),

                            // substations part:
                            new Pose3d(-0.66, 4.00177, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),
                            new Pose3d(15.151354, 4.00177, 0, new Rotation3d(0, 0, degreesToRadians(0))),

                            // blue part:
                            new Pose3d(14.486128, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(0))),
                            new Pose3d(14.486128, 0, 0, new Rotation3d(0, 0, degreesToRadians(0))),
                            new Pose3d(14.486128, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(0)))
                    ));
        }

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            System.out.println("Adjusting for BLUE.");
            targetPoses = Collections.unmodifiableList(
                    List.of(
                            // red part:
                            new Pose3d(-14.486128, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(0))),
                            new Pose3d(-14.486128, 0, 0, new Rotation3d(0, 0, degreesToRadians(0))),
                            new Pose3d(-14.486128, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(0))),

                            // substations part:
                            new Pose3d(15.151354, 4.00177, 0, new Rotation3d(0, 0, degreesToRadians(0))),
                            new Pose3d(-0.66, -4.00177, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),

                            // blue part:
                            new Pose3d(0, -1.67, 0.0, new Rotation3d(0, 0, degreesToRadians(180.0))),
                            new Pose3d(0, 0, 0, new Rotation3d(0, 0, degreesToRadians(180.0))),
                            new Pose3d(0, 1.67, 0, new Rotation3d(0, 0, degreesToRadians(180.0)))
                    ));
        }
    }

    public Boolean ShouldAcceptMeasurement(Pose2d visionPose) {
        Pose2d currentPose = getCurrentPose();
        double acceptanceZone = 1.5; // in meters

        if (Math.abs(currentPose.getX()) > acceptanceZone && Math.signum(visionPose.getX()) != Math.signum(currentPose.getX())) {
            return false;
        }

        if (Math.abs(currentPose.getY()) > acceptanceZone && Math.signum(visionPose.getY()) != Math.signum(currentPose.getY())) {
            return false;
        }

        return true;
    }

    @Override
    public void periodic() {
        // Update pose estimator with the best visible target
        var pipelineResult = photonCamera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        SmartDashboard.putNumber("pipeline index", photonCamera.getPipelineIndex());

        SmartDashboard.putNumber("odom angle", getCurrentPose().getRotation().getDegrees());

        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets() && photonCamera.getPipelineIndex() == 0) {
            previousPipelineTimestamp = resultTimestamp;
            var target = pipelineResult.getBestTarget();
            var fiducialId = target.getFiducialId();
            if (target.getPoseAmbiguity() <= .1 && fiducialId >= 0 && ((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue && fiducialId >= 6) || (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red && fiducialId <= 3))) {
                counter += 1;
                average = counter / stopWatch.getDuration();

                // var targetPose = targetPoses.get(0);
                var targetPose = targetPoses.get(fiducialId - 1);
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
                Pose2d visionPose = visionMeasurement.toPose2d();
                visionPose = new Pose2d(-visionPose.getY(), -visionPose.getX(), Rotation2d.fromDegrees(-visionPose.getRotation().getDegrees()));
                this.m_visionPose = visionPose;

                SmartDashboard.putNumber("vision x: ", visionPose.getX());
                SmartDashboard.putNumber("vision y: ", visionPose.getY());

                Pose3d cameraDistance = targetPose.transformBy(camToTarget.inverse());
                cameraDistance = cameraDistance.transformBy(CAMERA_TO_ROBOT);
                Pose2d axisedDistances = new Pose2d(-cameraDistance.getY(), -cameraDistance.getX(), new Rotation2d());

                poseEstimator.RanTest(axisedDistances, resultTimestamp);

                if (!DriverStation.isAutonomous() && (Math.abs(visionPose.getY())) < 0.7 && 2 < getCurrentPose().getY() && getCurrentPose().getY() < 6) {
                    poseEstimator.addVisionMeasurementWithoutFilter(visionPose, resultTimestamp);
                } else if (!DriverStation.isAutonomous() && (stopWatch.getDuration() - lastVisionTime > 4)) {
                    poseEstimator.addVisionMeasurementWithoutFilter(visionPose, resultTimestamp);
                } else {
                    // vision pose, getCurrentPose()
                    // if (DriverStation.isAutonomous() && visionPose.getTranslation().getDistance(getCurrentPose().getTranslation()) < 0.7)

                    // if (DriverStation.isAutonomous() && Math.abs(visionPose.getX() - getCurrentPose().getX()) < 0.7)
                    // poseEstimator.addVisionMeasurement(visionPose, resultTimestamp);
                    // else if (!DriverStation.isAutonomous())
                    // poseEstimator.addVisionMeasurement(visionPose, resultTimestamp);

                    poseEstimator.addVisionMeasurement(visionPose, resultTimestamp);
                }

                lastVisionTime = stopWatch.getDuration();
            }
        }
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(Rotation2d.fromDegrees(-drivetrainSubsystem.gyro.getYaw()), drivetrainSubsystem.getModulePositions());

        field2d.setRobotPose(convertToField2d(getCurrentPose()));

        Logger.getInstance().recordOutput("RobotPoses/MeasuredPose", convertToField2d(getCurrentPose()));

        Logger.getInstance().recordOutput("Vision/counter", counter);
        Logger.getInstance().recordOutput("Vision/average", average);
        Logger.getInstance().recordOutput("Vision/limelight_connected", photonCamera.isConnected());

        cameraConnected.setBoolean(photonCamera.isConnected());
    }

    public void resetCounter() {
        this.counter = 0;
    }

    public static Pose2d convertToField2d(Pose2d currentLocation) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return new Pose2d(currentLocation.getY() + 1.02, currentLocation.getX() + 2.74, Rotation2d.fromDegrees(currentLocation.getRotation().getDegrees() + 180));
        } else {
            return new Pose2d(-currentLocation.getY() + 15.494, -currentLocation.getX() + 2.74, currentLocation.getRotation());
        }
    }

    public Pose2d getTargetPose() {
        return poseEstimator.targetPose;
    }

    public Boolean hasTargets() {
        return photonCamera.getLatestResult().hasTargets();
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getRegularAxisPose() {
        Pose2d pose = getCurrentPose();
        return new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(-drivetrainSubsystem.gyro.getYaw()));
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called when the robot's
     * position on the field is known, like at the beginning of a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                Rotation2d.fromDegrees(-drivetrainSubsystem.gyro.getYaw()),
                drivetrainSubsystem.getModulePositions(),
                newPose
        );
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets what
     * "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

}
