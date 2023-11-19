package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {
    public static Pose2d TARGET_APRIL = new Pose2d(0, 0, new Rotation2d(0));

    public static double FLYWHEEL_P = 0.5;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 0.0010570821;
    private static double[][] FlyWheelManualRPM = {
        {1.0, 2050},
        {1.25, 2050}
    };
    public static MyTreeMap FLYWHEEL_RPM_MAP = new MyTreeMap();

    static {
        for (double[] pair : FlyWheelManualRPM) {
            FLYWHEEL_RPM_MAP.put(pair[0], pair[1]);
        }
    }

    public static double HOOD_P = 0.1;
    public static double HOOD_I = 1e-4;
    public static double HOOD_D = 1;
    public static double HOOD_F = 0; //0.0010570820886868473
    private static double[][] HoodManualAngle = {
        {1.0, 11.0},
        {1.25, 12.0},
        {1.5, 13.0},
        {1.75, 14.0},
        {2, 15.0},
        {2.25, 16.0},
        {2.75, 17.0}
    };
    public static MyTreeMap HOOD_ANGLE_MAP = new MyTreeMap();

    static {
        for (double[] pair : HoodManualAngle) {
            HOOD_ANGLE_MAP.put(pair[0], pair[1]);
        }
    }
}
