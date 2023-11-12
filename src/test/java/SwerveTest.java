import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import org.junit.jupiter.api.Test;

public class SwerveTest {    
    public ChassisSpeeds skew_calculation(ChassisSpeeds setpoint) {
        return skew_calculation(setpoint, 0.12);
    }

    public ChassisSpeeds skew_calculation(ChassisSpeeds setpoint, double loop) {
        var setpointTwist = new Pose2d().log(new Pose2d(setpoint.vxMetersPerSecond * loop, setpoint.vyMetersPerSecond * loop, new Rotation2d(setpoint.omegaRadiansPerSecond * loop)));
        var adjustedSpeeds = new ChassisSpeeds(setpointTwist.dx / loop, setpointTwist.dy / loop, setpointTwist.dtheta / loop);

        SmartDashboard.putNumber("x change: ", (adjustedSpeeds.vxMetersPerSecond - setpoint.vxMetersPerSecond));
        SmartDashboard.putNumber("y change: ", (adjustedSpeeds.vyMetersPerSecond - setpoint.vxMetersPerSecond));

        return adjustedSpeeds;
    }

    public static final double DELTA = 1e-2; // acceptable deviation range

    public static void assertEquals(double a, double b, double delta) {
        assert Math.abs(a - b) < delta;
    }

    public static void assertNotEquals(double a, double b, double delta) {
        assert Math.abs(a - b) > delta;
    }

    @Test // marks this method as a test
    public void swerve_skew_calculation_test() {
        System.out.println("------testing-------");
        var desired_chassis_speeds = new ChassisSpeeds(0, 2, Math.toRadians(85));
        var after_calculation_speeds = skew_calculation(desired_chassis_speeds);
        System.out.println("Vx_delta: " + (after_calculation_speeds.vxMetersPerSecond - desired_chassis_speeds.vxMetersPerSecond) + " Vy_delta: " + (after_calculation_speeds.vyMetersPerSecond - desired_chassis_speeds.vyMetersPerSecond) + " omega_theta: " + (after_calculation_speeds.omegaRadiansPerSecond - desired_chassis_speeds.omegaRadiansPerSecond));

        assertNotEquals(after_calculation_speeds.vxMetersPerSecond, desired_chassis_speeds.vxMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.vyMetersPerSecond, desired_chassis_speeds.vyMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.omegaRadiansPerSecond, desired_chassis_speeds.omegaRadiansPerSecond, DELTA);

        // Second test:
        desired_chassis_speeds = new ChassisSpeeds(0, 1, Math.toRadians(100));
        after_calculation_speeds = skew_calculation(desired_chassis_speeds);
        System.out.println("\nSecond test: \n" + "Vx_delta: " + (after_calculation_speeds.vxMetersPerSecond - desired_chassis_speeds.vxMetersPerSecond) + " Vy_delta: " + (after_calculation_speeds.vyMetersPerSecond - desired_chassis_speeds.vyMetersPerSecond) + " omega_theta: " + (after_calculation_speeds.omegaRadiansPerSecond - desired_chassis_speeds.omegaRadiansPerSecond));

        assertNotEquals(after_calculation_speeds.vxMetersPerSecond, desired_chassis_speeds.vxMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.vyMetersPerSecond, desired_chassis_speeds.vyMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.omegaRadiansPerSecond, desired_chassis_speeds.omegaRadiansPerSecond, DELTA);

        // Third test:
        desired_chassis_speeds = new ChassisSpeeds(0, 2, Math.toRadians(85));
        after_calculation_speeds = skew_calculation(desired_chassis_speeds, 0.02);
        System.out.println("\nthird test: " + "Vx_delta: " + (after_calculation_speeds.vxMetersPerSecond - desired_chassis_speeds.vxMetersPerSecond) + " Vy_delta: " + (after_calculation_speeds.vyMetersPerSecond - desired_chassis_speeds.vyMetersPerSecond) + " omega_theta: " + (after_calculation_speeds.omegaRadiansPerSecond - desired_chassis_speeds.omegaRadiansPerSecond));

        assertNotEquals(after_calculation_speeds.vxMetersPerSecond, desired_chassis_speeds.vxMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.vyMetersPerSecond, desired_chassis_speeds.vyMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.omegaRadiansPerSecond, desired_chassis_speeds.omegaRadiansPerSecond, DELTA);

        // Symmetric test:
        desired_chassis_speeds = new ChassisSpeeds(2, 0, Math.toRadians(85));
        after_calculation_speeds = skew_calculation(desired_chassis_speeds);
        System.out.println("\nSymmetric test: " + "Vx_delta: " + (after_calculation_speeds.vxMetersPerSecond - desired_chassis_speeds.vxMetersPerSecond) + " Vy_delta: " + (after_calculation_speeds.vyMetersPerSecond - desired_chassis_speeds.vyMetersPerSecond) + " omega_theta: " + (after_calculation_speeds.omegaRadiansPerSecond - desired_chassis_speeds.omegaRadiansPerSecond));

        assertEquals(after_calculation_speeds.vxMetersPerSecond, desired_chassis_speeds.vxMetersPerSecond, DELTA);
        assertNotEquals(after_calculation_speeds.vyMetersPerSecond, desired_chassis_speeds.vyMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.omegaRadiansPerSecond, desired_chassis_speeds.omegaRadiansPerSecond, DELTA);

        // Sanity test:
        desired_chassis_speeds = new ChassisSpeeds(0, 2, Math.toRadians(0));
        after_calculation_speeds = skew_calculation(desired_chassis_speeds);
        System.out.println("\nSanity test: " + "Vx_delta: " + (after_calculation_speeds.vxMetersPerSecond - desired_chassis_speeds.vxMetersPerSecond) + " Vy_delta: " + (after_calculation_speeds.vyMetersPerSecond - desired_chassis_speeds.vyMetersPerSecond) + " omega_theta: " + (after_calculation_speeds.omegaRadiansPerSecond - desired_chassis_speeds.omegaRadiansPerSecond));

        assertEquals(after_calculation_speeds.vxMetersPerSecond, desired_chassis_speeds.vxMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.vyMetersPerSecond, desired_chassis_speeds.vyMetersPerSecond, DELTA);
        assertEquals(after_calculation_speeds.omegaRadiansPerSecond, desired_chassis_speeds.omegaRadiansPerSecond, DELTA);
    }
}