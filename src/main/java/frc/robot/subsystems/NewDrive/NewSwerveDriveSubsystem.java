package frc.robot.subsystems.NewDrive;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.time.StopWatch;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.SwerveConstants;
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;

public class NewSwerveDriveSubsystem extends TimeMeasurementSubsystem {
    private static NewSwerveDriveSubsystem instance = null;
 
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
    
    public static NewSwerveDriveSubsystem getInstance() {
        if (instance == null)
            instance = NewSwerveDriveSubsystem.getDefaultSwerve();
        return instance;
    }
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
        double homeFrontLeftAngle = 84.81; // old 82
        double homeFrontRightAngle = 15.73; // old 22
        double homeBackLeftAngle = 312.36; // old 314
        double homeBackRightAngle = 128.75; // old 131



        var leftFront = new SwerveModuleFalcon500(
                14, 24,4, 
                -Units.degreesToRadians(homeFrontLeftAngle)
        );

        var rightFront = new SwerveModuleFalcon500(
                11, 21, 1, 
                -Units.degreesToRadians(homeFrontRightAngle)
        );

        var leftRear = new SwerveModuleFalcon500(
                13, 23, 3, 
                -Units.degreesToRadians(homeBackLeftAngle)
        );

        var rightRear = new SwerveModuleFalcon500(
                12, 22, 2,
                 -Units.degreesToRadians(homeBackRightAngle)
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
        var loop = 0.15;
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
    public void _periodic() {
        for (int i = 0; i < swerveModules.length; i++) {
            currentModuleStates[i] = swerveModules[i].getState();
            currentPositions[i] = swerveModules[i].getPosition();
            swerveModules[i].resetToAbsolute();
        }

        var after_skew_velocity = skew_calculation(wantedRobotVelocity);
        wantedModuleStates = this.kinematics.toSwerveModuleStates(after_skew_velocity);

        Logger.recordOutput("current module states", currentModuleStates);
        Logger.recordOutput("wanted module states", wantedModuleStates);
    
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

            Logger.recordOutput("ModuleStates/Current/" + i + "/angle", current_module_state.angle.getDegrees());
            Logger.recordOutput("ModuleStates/Current/" + i + "/velocity", current_module_state.speedMetersPerSecond);

            Logger.recordOutput("ModuleStates/Wanted/" + i + "/angle", wantedModuleStates[i].angle.getDegrees());
            Logger.recordOutput("ModuleStates/Wanted/" + i + "/velocity", wantedModuleStates[i].speedMetersPerSecond);
        }
    }

    public double getYawDegrees() {
        return pigeon2.getYaw();
    }

    public void getAllCanCoders() {
        for (SwerveModuleFalcon500 module : this.swerveModules) {
            module.steerEncoder.configMagnetOffset(0);
        }

        for (int i = 0; i < swerveModules.length; i++) {
            var nt_publisher = NetworkTableInstance.getDefault().getTable("CANCoders").getDoubleTopic("CANCoder "  + i).publish();
            nt_publisher.set(swerveModules[i].steerEncoder.getAbsolutePosition());
        }
    }

    public void printAllCANCoders() {
        // [frontLeft, frontRight, backLeft, backRight]
        System.out.println("\n\n");

        System.out.println("FRONT LEFT CANCODER: " + this.swerveModules[0].steerEncoder.getAbsolutePosition());
        System.out.println("FRONT RIGHT CANCODER: " + this.swerveModules[1].steerEncoder.getAbsolutePosition());
        System.out.println("BACK LEFT CANCODER: " + this.swerveModules[2].steerEncoder.getAbsolutePosition());
        System.out.println("BACK RIGHT CANCODER: " + this.swerveModules[3].steerEncoder.getAbsolutePosition());

        System.out.println("\n\n");
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

    public Command getDefaultPathFollowingCommand(PathPlannerPath path, NewPoseEstimatorSubsystem poseEstimatorSubsystem) {
        return new FollowPathHolonomic(
                path,
                poseEstimatorSubsystem::getCurrentPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setRelativeVelocities, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.91, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                this // Reference to this subsystem to set requirements
        );
    }
}

