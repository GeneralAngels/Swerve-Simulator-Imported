package frc.robot.subsystems.NewDrive;


import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.time.StopWatch;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.SwerveConstants;


public class NewSwerveDriveSubsystem extends SubsystemBase {
    SwerveDriveKinematics kinematics;

    SwerveModuleFalcon500[] swerveModules;

    ChassisSpeeds wantedRobotVelocity = new ChassisSpeeds();


    SwerveModuleState[] wantedModuleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    SwerveModuleState[] currentModuleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    SwerveModulePosition[] currentPositions = new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    public Pigeon2 pigeon2;
    public BasePigeonSimCollection pigeonSimCollection;

    boolean limitingRotatingMaxVel = false;

    StopWatch simStopWatch = new StopWatch();

    /**
     *
     * @param swerveModules: the swerve modules list = [frontLeft, frontRight, backLeft, backRight]
     */
    public NewSwerveDriveSubsystem(SwerveModuleFalcon500[] swerveModules, Pigeon2 pigeon2) {
        var frontLeftLocation = new Translation2d(SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2);
        var frontRightLocation = new Translation2d(SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2);
        var backLeftLocation = new Translation2d(-SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2);
        var backRightLocation = new Translation2d(-SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2);

        kinematics =
                new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        this.swerveModules = swerveModules;

        for (SwerveModuleFalcon500 module : swerveModules) {
            module.resetToAbsolute();
        }

        simStopWatch.start();

        this.pigeon2 = pigeon2;
        this.pigeonSimCollection = pigeon2.getSimCollection();
    }

    public static NewSwerveDriveSubsystem getDefaultSwerve() {
        var leftFront = new SwerveModuleFalcon500(
                14, 24,4, -Units.degreesToRadians(SwerveConstants.homeFrontLeftAngle)
        );

        var rightFront = new SwerveModuleFalcon500(
                11, 21, 1, -Units.degreesToRadians(SwerveConstants.homeFrontRightAngle)
        );

        var leftRear = new SwerveModuleFalcon500(
                13, 23, 3, -Units.degreesToRadians(SwerveConstants.homeRearLeftAngle)
        );

        var rightRear = new SwerveModuleFalcon500(
                12, 22, 2, -Units.degreesToRadians(SwerveConstants.homeRearRightAngle)
        );

        var pigeon2 = new Pigeon2(30);

        return new NewSwerveDriveSubsystem(new SwerveModuleFalcon500[] {leftFront, rightFront, leftRear, rightRear}, pigeon2);
    }

    public void setRelativeVelocities(ChassisSpeeds relativeVelocities) {
        this.wantedRobotVelocity = relativeVelocities;
    }

    public void setAbsoluteVelocities(ChassisSpeeds absoluteVelocities) {
        this.wantedRobotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                absoluteVelocities, Rotation2d.fromDegrees(getYawDegrees())
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.kinematics.toChassisSpeeds(currentModuleStates);
    }

    public SwerveModulePosition[] getModulesPosition() {
        return currentPositions;
    }

    public ChassisSpeeds skew_calculation(ChassisSpeeds setpoint) {
        var loop = 0.12;
        var setpointTwist =
                new Pose2d()
                        .log(
                                new Pose2d(
                                        setpoint.vxMetersPerSecond * loop,
                                        setpoint.vyMetersPerSecond * loop,
                                        new Rotation2d(setpoint.omegaRadiansPerSecond * loop)));
        var adjustedSpeeds =
                new ChassisSpeeds(
                        setpointTwist.dx / loop,
                        setpointTwist.dy / loop,
                        setpointTwist.dtheta / loop);


        return adjustedSpeeds;
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < swerveModules.length; i++) {
            currentModuleStates[i] = swerveModules[i].getState();
            currentPositions[i] = swerveModules[i].getPosition();
        }

        var after_skew_velocity = skew_calculation(wantedRobotVelocity);
        wantedModuleStates = this.kinematics.toSwerveModuleStates(after_skew_velocity);
        System.out.println(wantedModuleStates[0].speedMetersPerSecond);


        SmartDashboard.putNumber("pigeon angle", pigeon2.getYaw());

        if (limitingRotatingMaxVel) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    currentModuleStates, getChassisSpeeds(),
                    SwerveConstants.maxSpeed,
                    4.8, 4.2
            );
        }
        else {
            SwerveDriveKinematics.desaturateWheelSpeeds(wantedModuleStates, SwerveConstants.maxSpeed);
        }

        for (int i = 0; i < swerveModules.length; i++) {
            var current_module_state = swerveModules[i].getState();
            wantedModuleStates[i] = SwerveModuleState.optimize(wantedModuleStates[i], current_module_state.angle);
            wantedModuleStates[i].angle = Rotation2d.fromDegrees(placeInAppropriate0To360Scope(current_module_state.angle.getDegrees(), wantedModuleStates[i].angle.getDegrees()));

            swerveModules[i].setState(wantedModuleStates[i]);
        }
    }

    public double getYawDegrees() {
        return pigeon2.getYaw();
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Update simulation encoders in drive motors.
        double looperDt = simStopWatch.getDuration();
        simStopWatch.start();

        for (SwerveModuleFalcon500 module : swerveModules) {
            module.updateSim(looperDt);
        }

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].showSimOutputs(i);
        }

        var current_swerve_speed = getChassisSpeeds();
        // pigeonSimCollection.addHeading(Units.radiansToDegrees(current_swerve_speed.omegaRadiansPerSecond) * looperDt);
        pigeonSimCollection.setRawHeading(pigeon2.getYaw() + Units.radiansToDegrees(current_swerve_speed.omegaRadiansPerSecond) * looperDt);

        SmartDashboard.putNumber("x speed", current_swerve_speed.vxMetersPerSecond);
        SmartDashboard.putNumber("y speed", current_swerve_speed.vyMetersPerSecond);
        SmartDashboard.putNumber("omega radians per second", current_swerve_speed.omegaRadiansPerSecond);
    }
}

