import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Robot;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class GenericTest {
    public static final double DELTA = 1e-2; // acceptable deviation range
    public static final double a = 150.0 / 7;

    private static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double DRIVE_GEAR_RATIO_SECONDARY = (50.0 / 24.0) * (19.0 / 25.0) * (45.0 / 15.0);


    public GenericTest() {

    }

    @Test
    public void some_test() {
        System.out.println("running");
        assert 1 == 1;
    }

    @Test
    public void some() {
        System.out.println(DRIVE_GEAR_RATIO);
        System.out.println(DRIVE_GEAR_RATIO_SECONDARY);
    }
}