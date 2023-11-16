package frc.robot.subsystems;

import com.ctre.phoenix.time.StopWatch;
import com.revrobotics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    public CANSparkMax m_flywheel_motor;
    private static final int deviceId = 1;
    public SparkMaxPIDController m_flywheel_pidController;
    private RelativeEncoder m_encoder;

    double Kp = 0.5;
    double Ki = 0.0;
    double Kd = 0.0;
    double Kf = 946.0003255208334;

    public double desiredVelocity = 0.0;

    public Shooter() {
        m_flywheel_motor = new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_flywheel_motor.restoreFactoryDefaults();

        m_flywheel_pidController = m_flywheel_motor.getPIDController();
        m_encoder = m_flywheel_motor.getEncoder();

        m_flywheel_pidController.setP(Kp);
        m_flywheel_pidController.setI(Ki);
        m_flywheel_pidController.setD(Kd);
        m_flywheel_pidController.setFF(Kf);
        m_flywheel_pidController.setOutputRange(-1, 1);

        m_flywheel_motor.burnFlash();

        REVPhysicsSim.getInstance().addSparkMax(m_flywheel_motor, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        m_flywheel_pidController.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity); // We can use smart velocity if we want.
        Logger.recordOutput("shooter velocity", m_flywheel_motor.getEncoder().getVelocity());
    }

    public void setDesiredVelocity(double velocity) {
        desiredVelocity = velocity;
    }

    public boolean atDesiredVelocity() {
        return Math.abs(m_encoder.getVelocity() - desiredVelocity) <= 0.05;
    }

    public double getKf() {
        m_flywheel_motor.setVoltage(6);

        StopWatch stopWatch = new StopWatch();
        stopWatch.start();

        while (stopWatch.getDuration() < 5) {
            Logger.recordOutput("Shooter velocity", m_flywheel_motor.getEncoder().getVelocity());
        }

        return m_flywheel_motor.getEncoder().getVelocity() / 6;
    }
}
