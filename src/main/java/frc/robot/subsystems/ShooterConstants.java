package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {
    public static Pose2d TARGET_APRIL = new Pose2d(0, 0, new Rotation2d(0));

    public static double FLYWHEEL_P = 0.00007;
    public static double FLYWHEEL_I = 0.000000006;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 0.0001620370333740905;
    private static double[][] FlyWheelManualRPM = {
            {0, 120},
            {1.0, 2050},
            {1.25, 2050},
            {1.5, 2100},
            {1.75, 2100},
            {2.0, 2150},
            {2.25, 2150},
            {2.5, 2200}
    };
    public static MyTreeMap FLYWHEEL_RPM_MAP = new MyTreeMap();

    static {
        for (double[] pair : FlyWheelManualRPM) {
            FLYWHEEL_RPM_MAP.put(pair[0], pair[1]);
        }
    }

    public static double HOOD_P = 0.2;
    public static double HOOD_I = 1e-4;
    public static double HOOD_D = 1;
    public static double HOOD_F = 0.0; //0.0010570820886868473
    private static double[][] HoodManualAngle = {
            {1.0, 10.0},
            {1.25, 15.0},
            {1.5, 20.0},
            {1.75, 25.0},
            {2, 30.0},
            {2.25, 35.0},
            {2.75, 40.0}
    };
    public static MyTreeMap HOOD_ANGLE_MAP = new MyTreeMap();

    static {
        for (double[] pair : HoodManualAngle) {
            HOOD_ANGLE_MAP.put(pair[0], pair[1]);
        }
    }
}
