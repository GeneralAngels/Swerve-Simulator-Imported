// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Utils.Convertions;
import frc.robot.subsystems.Drive.Motors.falcon.Falcon;
import frc.robot.subsystems.Drive.Motors.falcon.RotationFalcon;

/**
 * Add your docs here.
 */
public class SwerveModule {
    public Falcon movementMotor;
    public RotationFalcon rotationMotor;

    double direction = 1;

    double RotationToMetersRatio; // how many rotations are one meter, rotation / 1 meter;

    double gearRatio = 6.75 / 1; // TODO: Change to constants!!!
    double wheelRadius = Convertions.inchesToMeters(4) / 2.0;

    public SwerveModule(Falcon movementMotor, RotationFalcon rotationMotor, double wheelRadius) {
        rotationMotor.setEncoder();
        this.movementMotor = movementMotor;
        this.rotationMotor = rotationMotor;

        movementMotor.setPosition(0);

        this.RotationToMetersRatio = 1 / (2 * Math.PI * wheelRadius) * 60;
    }

    public static SwerveModule fromPorts(int movementMotorPort, int rotationMotorPort, int canCoderPort, double homeAngle, double wheelRadius) {
        var movementMotor = new Falcon(new TalonFX(movementMotorPort), 0, 0.7, -0.7, SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd, SwerveConstants.Driving_MotorToDrivenRatio);
        var rotationMotor = new RotationFalcon(new TalonFX(rotationMotorPort), canCoderPort, 0, 0.6, -0.6, SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Kp, SwerveConstants.Rotation_Ki, SwerveConstants.Rotation_Kd, homeAngle, SwerveConstants.Rotation_MotorToDrivenRatio, true, true);

        return new SwerveModule(movementMotor, rotationMotor, wheelRadius);
    }

    public void setEncoder() {
        rotationMotor.setEncoder();
    }

    public double getAngle() {
        return rotationMotor.getAngleByFalcon();
    }

    public void setAngle(double angle) {
        rotationMotor.setAngle(angle);
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
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

    public static double[] optimize(double desiredAngle, double desiredVelocity, double currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredAngle);
        double targetSpeed = desiredVelocity;
        double delta = targetAngle - currentAngle;
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new double[]{targetAngle, targetSpeed};
    }

    public void setVelocity(double metersPerSecond) {
        movementMotor.setRpm(metersPerSecond * RotationToMetersRatio);
    }

    public void updatePositionToSim(double looperDt) {
        // TODO: Move to falcon class. It's all in the falcon part.
        var current_velocity = movementMotor.getVelocityTicks(); // in ticks / 100ms
        var current_velocity_in_seconds = current_velocity * 10; // in ticks / second
        int current_encoder_change = (int) (current_velocity_in_seconds * looperDt);

        movementMotor.sim_falcon.addIntegratedSensorPosition(current_encoder_change);
    }

    public double getVelocity() {
        // in meters / sec
        return movementMotor.getRpm() / RotationToMetersRatio;
    }

    public void setState(SwerveModuleState desiredState) {
        double[] optimizedStates = optimize(desiredState.angle.getDegrees(), desiredState.speedMetersPerSecond, getAngle());

        this.setVelocity(optimizedStates[1]);
        this.setAngle(optimizedStates[0]);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(Convertions.FalconTicksToMeters(this.movementMotor.getPosition(), gearRatio, wheelRadius), Rotation2d.fromDegrees(getAngle()));
    }


    public boolean isAllConnected() {
        Boolean driveMotorConnected = movementMotor._talon.getFaults(new Faults()) == ErrorCode.OK;
        Boolean rotationMotorConnected = rotationMotor._talon.getLastError() == ErrorCode.OK;
        Boolean canCoderConnected = rotationMotor.canCoder.getLastError() == ErrorCode.OK;

        return driveMotorConnected && rotationMotorConnected && canCoderConnected;
    }
}
