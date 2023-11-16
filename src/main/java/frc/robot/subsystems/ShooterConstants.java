package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static double FLYWHEEL_P = 0;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 0.0010570821;
    public static InterpolatingDoubleTreeMap FLYWHEEL_RPM_MAP;

    public static double HOOD_P = 0;
    public static double HOOD_I = 0;
    public static double HOOD_D = 0;
    public static InterpolatingDoubleTreeMap HOOD_ANGLE_MAP;
}
