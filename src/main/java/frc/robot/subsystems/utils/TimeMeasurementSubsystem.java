package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class TimeMeasurementSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
        double startTime = System.currentTimeMillis() / 1000.0;
        _periodic();
        double timeDelta = System.currentTimeMillis() / 1000.0  - startTime;

        Logger.recordOutput("Time Measurement/" + getName(), timeDelta);
    }

    public abstract void _periodic();
}
