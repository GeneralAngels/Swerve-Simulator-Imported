// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.Motors.falcon.Falcon;
import frc.robot.Constants.SwerveLimitationsConstants;
import frc.robot.subsystems.Drive.Motors.falcon.RotationFalcon;

/**
 * Add your docs here.
 */
public class SwerveContainer {
    // Setting DrivingMotors:
    Falcon drivingRightFront = new Falcon(
            new TalonFX(11),
            0,
            SwerveLimitationsConstants.maxVoltageMotor, -SwerveLimitationsConstants.maxVoltageMotor,
            SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
            SwerveConstants.Driving_MotorToDrivenRatio);

    Falcon drivingRightRear = new Falcon(
            new TalonFX(12),
            0,
            SwerveLimitationsConstants.maxVoltageMotor, -SwerveLimitationsConstants.maxVoltageMotor,
            SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
            SwerveConstants.Driving_MotorToDrivenRatio);

    Falcon drivingLeftRear = new Falcon(
            new TalonFX(13),
            0,
            SwerveLimitationsConstants.maxVoltageMotor, -SwerveLimitationsConstants.maxVoltageMotor,
            SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
            SwerveConstants.Driving_MotorToDrivenRatio);

    Falcon drivingLeftFront = new Falcon(
            new TalonFX(14),
            0,
            SwerveLimitationsConstants.maxVoltageMotor, -SwerveLimitationsConstants.maxVoltageMotor,
            SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
            SwerveConstants.Driving_MotorToDrivenRatio);

    // Setting Rotation Motors:
    RotationFalcon rotationRightFront = new RotationFalcon(
            new TalonFX(21), 1,
            0,
            0.6, -0.6,
            SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Kp, SwerveConstants.Rotation_Ki,
            SwerveConstants.Rotation_Kd,
            SwerveConstants.homeFrontRightAngle,
            SwerveConstants.Rotation_MotorToDrivenRatio,
            true, true);

    RotationFalcon rotationRightRear = new RotationFalcon(
            new TalonFX(22), 2,
            0,
            0.6, -0.6,
            SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Kp, SwerveConstants.Rotation_Ki,
            SwerveConstants.Rotation_Kd,
            SwerveConstants.homeRearRightAngle,
            SwerveConstants.Rotation_MotorToDrivenRatio,
            true, true);

    RotationFalcon rotationLeftRear = new RotationFalcon(
            new TalonFX(23), 3,
            0,
            0.6, -0.6,
            SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Kp, SwerveConstants.Rotation_Ki,
            SwerveConstants.Rotation_Kd,
            SwerveConstants.homeRearLeftAngle,
            SwerveConstants.Rotation_MotorToDrivenRatio,
            true, true);

    RotationFalcon rotationLeftFront = new RotationFalcon(
            new TalonFX(24), 4,
            0,
            0.6, -0.6,
            SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Kp, SwerveConstants.Rotation_Ki,
            SwerveConstants.Rotation_Kd,
            SwerveConstants.homeFrontLeftAngle,
            SwerveConstants.Rotation_MotorToDrivenRatio,
            true, true);

    SwerveModule moduleRightFront = new SwerveModule(drivingRightFront, rotationRightFront, SwerveConstants.wheelRadius);
    SwerveModule moduleRightRear = new SwerveModule(drivingRightRear, rotationRightRear, SwerveConstants.wheelRadius);
    SwerveModule moduleLeftFront = new SwerveModule(drivingLeftFront, rotationLeftFront, SwerveConstants.wheelRadius);
    SwerveModule moduleLeftRear = new SwerveModule(drivingLeftRear, rotationLeftRear, SwerveConstants.wheelRadius);

    // right front, right rear, left rear, left front

//    public SwerveDriveTrain swerve = new SwerveDriveTrain(
//            // moduleLeftFront, moduleRightFront, moduleRightRear, moduleLeftRear, // Switch rightRear and leftFront, rightFront and leftRear if rotation is opposite
//            // In swerve: RightFront, rightRear, LeftRear, LeftFront
//            moduleRightRear, moduleLeftRear, moduleLeftFront, moduleRightFront,
//            RobotContainer.pigeon2,
//            SwerveConstants.swerveWidth, SwerveConstants.swerveLength
//    );

    public WPI_PigeonIMU setAndReturnGyro() {
        WPI_PigeonIMU gyro = new WPI_PigeonIMU(30);
        return gyro;
    }
}
