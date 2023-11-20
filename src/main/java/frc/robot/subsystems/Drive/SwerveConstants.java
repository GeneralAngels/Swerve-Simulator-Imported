// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Add your docs here.
 */
public class SwerveConstants {
    public static final double maxSpeed = 4.8;

    // Swerve gears and wheels: 
    public static final double wheelRadius = 0.0508;
    public static final double Driving_MotorToDrivenRatio = 8.14 / 1;
    public static final double Rotation_MotorToDrivenRatio = 150.0 / 7.0;

    public static final double swerveWidth = 0.65;
    public static final double swerveLength = 0.65;

    // Driving PID Constants: 
    public static final double Drive_Kf = 0.045;
    public static final double Drive_Kp = 0.06;
    public static final double Drive_Ki = 0;
    public static final double Drive_Kd = 0;

    // Rotation PID Constants:
    public static final double Rotation_Kf = 0;
    public static final double Rotation_Kp = 0.09;
    public static final double Rotation_Ki = 0.0;
    public static final double Rotation_Kd = 0.0;


    /*
    Kinematics:
      mod0: right front
      mod1: right rear
      mod2: left rear
      mod3: left front
    
    Distances are by wpilib documentation and swerve width and length.
    The distances here are by the x axis positive direction (because of how we started to use the swerve).
    */
    public static Translation2d rightFrontLocation = new Translation2d(-SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2);
    public static Translation2d rightRearLocation = new Translation2d(SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2);
    public static Translation2d leftRearLocation = new Translation2d(SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2);
    public static Translation2d leftFrontLocation = new Translation2d(-SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2);

    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            rightFrontLocation,
            rightRearLocation,
            leftRearLocation,
            leftFrontLocation
    );

    /*
        BACK RIGHT CANCODER: 134.6484375
        BACK LEFT CANCODER: 196.611328125
        FRONT RIGHT CANCODER: 128.84765625
        FRONT LEFT CANCODER: 175.517578125
     */

    // CanCoder home angles:
    // TODO: Copy from driver station laptop
    public static double homeFrontRightAngle = 128.84;
    public static double homeRearRightAngle = 134.64;
    public static double homeRearLeftAngle = 196.61;
    public static double homeFrontLeftAngle = 91.14;


//    public static double homeFrontRightAngle = (39.11) % 360;
//    public static double homeRearRightAngle = (225.52) % 360;
//    public static double homeRearLeftAngle = (15.20) % 360;
//    public static double homeFrontLeftAngle = (84.19) % 360;

    public static double homeSpearFrontRightAngle = 83.67 + 90 + 180;
    public static double homeSpearRearRightAngle = 340.48 + 90 + 180;
    public static double homeSpearRearLeftAngle = 65.1 + 90;
    public static double homeSpearFrontLeftAngle = 235.1 + 90;
}