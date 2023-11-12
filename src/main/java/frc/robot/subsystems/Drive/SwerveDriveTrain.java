// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.time.StopWatch;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveTrain extends SubsystemBase {
    public SwerveModule rightFront;
    public SwerveModule rightRear;
    public SwerveModule leftRear;
    public SwerveModule leftFront;

    public Pigeon2 gyro;
    public BasePigeonSimCollection gyroSimCollection;

    double width;
    double length;

    double desaturatedCounter = 0;

    double lastTime = WPIUtilJNI.now() * 1.0e-6;

    /**
     * Creates a new SwerveDriveTrain.
     */

    /*
     * When resting the absolute encoders, the yellow part of the robot needs to point the positive x
     * direction. Positive x' towards the yellow part of the swerve. Resting the home angle encoders
     * need to according to this logic.
     */
    public SwerveDriveTrain(SwerveModule rightFront, SwerveModule rightRear, SwerveModule leftRear,
                            SwerveModule leftFront, Pigeon2 gyro, double width, double length) {

        this.rightFront = rightFront;
        this.rightRear = rightRear;
        this.leftRear = leftRear;
        this.leftFront = leftFront;

        this.gyro = gyro;
        this.gyroSimCollection = gyro.getSimCollection();
        this.gyro.zeroGyroBiasNow();
        this.gyro.setYaw(0);

        System.out.println("after calibration");

        this.width = width;
        this.length = length;

        if (Robot.isSimulation()) {
            lastTime = WPIUtilJNI.now() * 1.0e-6;
        }
    }

    public void setWpiRelativeSwerveVelocoties(ChassisSpeeds speeds) {
        speeds = skew_calculation(speeds);

        SwerveModuleState[] swerveModuleStates =
                SwerveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(speeds.vxMetersPerSecond,
                        -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond));

        SwerveModuleState[] unDesaturated = new SwerveModuleState[]{swerveModuleStates[0], swerveModuleStates[1], swerveModuleStates[2], swerveModuleStates[3]};

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (int i = 0; i < 4; i++) {
            if (swerveModuleStates[i] != unDesaturated[i]) {
                desaturatedCounter++;
                break;
            }
        }

        this.rightFront.setState(swerveModuleStates[0]);
        this.rightRear.setState(swerveModuleStates[1]);
        this.leftRear.setState(swerveModuleStates[2]);
        this.leftFront.setState(swerveModuleStates[3]);

        // Logging:
        Logger.getInstance().recordOutput("Swerve/States/DesiredStates", swerveModuleStates);
    }

    public void lockWheels(double speed) {
        rightFront.setState(new SwerveModuleState(speed, Rotation2d.fromDegrees(180 - 45 + 90)));
        leftFront.setState(new SwerveModuleState(speed, Rotation2d.fromDegrees(360 - (180 - 45) + 90)));
        rightRear.setState(new SwerveModuleState(speed, Rotation2d.fromDegrees(45 + 90)));
        leftRear.setState(new SwerveModuleState(speed, Rotation2d.fromDegrees(360 - 45 + 90)));
    }

    public void resetAllEncoderToAbsolute() {
        this.rightFront.setEncoder();
        this.rightRear.setEncoder();
        this.leftRear.setEncoder();
        this.leftFront.setEncoder();
    }

    public void restSwerve() {
        rightFront.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(rightFront.getAngle())));
        rightRear.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(rightRear.getAngle())));
        leftRear.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(leftRear.getAngle())));
        leftFront.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(leftFront.getAngle())));
    }

    public double getOmegaCorrection(double setpoint) {
        return getOmegaCorrection(setpoint, 1.5);
    }

    public double getOmegaCorrection(double setpoint, double omegaKp) {
        double currentAngle = -gyro.getYaw();
        setpoint = placeInAppropriate0To360Scope(currentAngle, setpoint);

        double omega = Math.toRadians((setpoint - currentAngle) * omegaKp);
        return -omega;
    }

    public void setWpiAbsoluteVelocoties(ChassisSpeeds speeds) {
        double angle = -this.gyro.getYaw();
        this.setWpiRelativeSwerveVelocoties(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                Rotation2d.fromDegrees(Math.abs(angle) % 360 * -Math.signum(angle))));
    }

    public SwerveModuleState getRightFrontModuleState() {
        return new SwerveModuleState(this.rightFront.getVelocity(),
                Rotation2d.fromDegrees(this.rightFront.getAngle()));
    }

    public SwerveModuleState getRightRearModuleState() {
        return new SwerveModuleState(this.rightRear.getVelocity(),
                Rotation2d.fromDegrees(this.rightRear.getAngle()));
    }

    public SwerveModuleState getLeftFrontModuleState() {
        return new SwerveModuleState(this.leftFront.getVelocity(),
                Rotation2d.fromDegrees(this.leftFront.getAngle()));
    }

    public SwerveModuleState getLeftRearModuleState() {
        return new SwerveModuleState(this.leftRear.getVelocity(),
                Rotation2d.fromDegrees(this.leftRear.getAngle()));
    }

    public ChassisSpeeds getSwerveSpeeds() {
        return SwerveConstants.kinematics.toChassisSpeeds(
                getRightFrontModuleState(),
                getRightRearModuleState(),
                getLeftRearModuleState(),
                getLeftFrontModuleState()
        );
    }

    public double getSpeed() {
        ChassisSpeeds speeds = getSwerveSpeeds();

        return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                rightFront.getModulePosition(),
                rightRear.getModulePosition(),
                leftRear.getModulePosition(),
                leftFront.getModulePosition()
        };
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

    public ChassisSpeeds skew_calculation(ChassisSpeeds setpoint) {
        // TODO: What is the time change between each call.
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

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("rightFront velocity", rightFront.getVelocity());

        double looperDt = WPIUtilJNI.now() * 1.0e-6 - lastTime;
        lastTime = WPIUtilJNI.now() * 1.0e-6;

        rightFront.updatePositionToSim(looperDt);
        rightRear.updatePositionToSim(looperDt);
        leftRear.updatePositionToSim(looperDt);
        leftFront.updatePositionToSim(looperDt);

        var current_speeds = getSwerveSpeeds();
        gyroSimCollection.setRawHeading(gyro.getYaw() + Math.toDegrees(current_speeds.omegaRadiansPerSecond) * looperDt);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        Logger.getInstance().recordOutput("a/b", 3);
        Logger.getInstance().recordOutput("a/c", 4);

        // Issues detection:
        // To know if a device is connected, we use the function device.getLastError().
        SmartDashboard.putBoolean("right-front module connected",
                rightFront.movementMotor._talon.getLastError() == ErrorCode.OK &&
                        rightFront.rotationMotor._talon.getLastError() == ErrorCode.OK &&
                        rightFront.rotationMotor.canCoder.getLastError() == ErrorCode.OK);

        // Logger.getInstance().recordOutput("Swerve/States/MeasuredStates", new SwerveModuleState[] {getRightFrontModuleState(), getRightRearModuleState(), getLeftRearModuleState(), getLeftFrontModuleState()});

        // Logger.getInstance().recordOutput("Swerve/States/MeasuredStates", new SwerveModuleState[] {
        //   getRightFrontModuleState(), // real - rightdown
        //   getLeftRearModuleState(),      //real - lefRear - change leftFront
        //   getRightRearModuleState(), //real - rightfront, here leftdown
        //   getLeftFrontModuleState() // real leftup
        // });


        // Logger.getInstance().recordOutput("Swerve/States/MeasuredStates", new SwerveModuleState[] {
        //   getLeftRearModuleState(), // real rightdown
        //   getLeftFrontModuleState(),      //, //real - lefRear - change leftFront
        //   getRightFrontModuleState(), //real - rightfront, here leftdown
        //   getRightRearModuleState() // real leftup
        // });


        Logger.getInstance().recordOutput("Swerve/States/RightFrontReal", getLeftFrontModuleState().angle.getDegrees());
        Logger.getInstance().recordOutput("Swerve/States/LeftFrontReal", getLeftRearModuleState().angle.getDegrees());
        Logger.getInstance().recordOutput("Swerve/States/RightRearReal", getRightFrontModuleState().angle.getDegrees());
        Logger.getInstance().recordOutput("Swerve/States/LeftRearReal", getRightRearModuleState().angle.getDegrees());

        SwerveModuleState rightFrontReal = getLeftFrontModuleState();
        SwerveModuleState rightRearReal = getRightFrontModuleState();
        SwerveModuleState leftRearReal = getRightRearModuleState();
        SwerveModuleState leftFrontReal = getLeftRearModuleState();

        // System.out.println("\n movmentMotor: " + leftFront.movementMotor._talon.getLastError());
        // System.out.println("\n rotationMotor: " + leftFront.rotationMotor._talon.getLastError());

        Logger.getInstance().recordOutput("Swerve/States/MeasuredStates", new SwerveModuleState[]{getLeftFrontModuleState(), leftFrontReal, rightRearReal, leftRearReal});

    }
}
