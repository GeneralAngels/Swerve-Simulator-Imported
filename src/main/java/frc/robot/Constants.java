// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.math.util.Units.degreesToRadians;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SwerveLimitationsConstants {
        public static final double maxVoltageMotor = 0.8;
    }

    public final static class VisionConstants {
        public final static Transform3d CAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(-0.33, 0.0, 0),
                        new Rotation3d(degreesToRadians(0), degreesToRadians(47), degreesToRadians(0)));

        public final static double Y_ANGLE_FOR_INTAKE = 350;
        public final static int CAMERA_WIDTH = 640;
        public final static int CAMERA_HEIGHT = 480;

        public static final double coneXkp = 0.02;
    }
}
