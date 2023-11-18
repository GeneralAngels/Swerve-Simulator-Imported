package frc.robot.subsystems;

import com.ctre.phoenix.time.StopWatch;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    public CANSparkMax m_flywheel_motor;
    private static final int flywheel_deviceId = 1;
    public SparkMaxPIDController m_flywheel_pidController;
    private RelativeEncoder m_flywheel_encoder;

    public CANSparkMax m_hood_motor;
    public SparkMaxPIDController m_hood_PidController;
    private RelativeEncoder m_hood_encoder;
    private static final int hood_deviceId = 2;


    public double desiredVelocity = 0.0;

    private static Shooter instance = null;

    public Shooter() {
        // Initializng the flywheel motor
        m_flywheel_motor = new CANSparkMax(flywheel_deviceId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_flywheel_motor.restoreFactoryDefaults();

        m_flywheel_pidController = m_flywheel_motor.getPIDController();
        m_flywheel_encoder = m_flywheel_motor.getEncoder();

        m_flywheel_pidController.setP(ShooterConstants.FLYWHEEL_P);
        m_flywheel_pidController.setI(ShooterConstants.FLYWHEEL_I);
        m_flywheel_pidController.setD(ShooterConstants.FLYWHEEL_D);
        m_flywheel_pidController.setFF(ShooterConstants.FLYWHEEL_F);
        m_flywheel_pidController.setOutputRange(-1, 1);

        m_flywheel_motor.burnFlash();

        // Initializing the hood motor
        m_hood_motor = new CANSparkMax(hood_deviceId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_hood_motor.restoreFactoryDefaults();

        m_hood_PidController = m_hood_motor.getPIDController();
        m_hood_encoder = m_hood_motor.getEncoder();

        m_hood_PidController.setP(ShooterConstants.HOOD_P);
        m_hood_PidController.setI(ShooterConstants.HOOD_I);
        m_hood_PidController.setD(ShooterConstants.HOOD_D);

        m_hood_motor.burnFlash();

        REVPhysicsSim.getInstance().addSparkMax(m_flywheel_motor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_hood_motor, DCMotor.getNEO(1));
    }

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    @Override
    public void periodic() {
        //m_flywheel_pidController.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity); // We can use smart velocity if we want.
        //Logger.recordOutput("shooter velocity", m_flywheel_motor.getEncoder().getVelocity());
        setHoodAngle();
    }

    public void setDesiredVelocity(double velocity) {
        desiredVelocity = velocity;
    }

    public boolean atDesiredVelocity() {
        return Math.abs(m_flywheel_encoder.getVelocity() - desiredVelocity) <= 0.05;
    }

    public void setHoodAngle() {
        Transform2d poseToTarget = new Transform2d(NewPoseEstimatorSubsystem.getInstance().getCurrentPose(), ShooterConstants.TARGET_APRIL);
        double distanceToTarget = Math.hypot(poseToTarget.getX(), poseToTarget.getY());

        distanceToTarget = 1.832;
        double hoodAngle = ShooterConstants.HOOD_ANGLE_MAP.get(distanceToTarget);
        m_hood_PidController.setReference(hoodAngle, CANSparkMax.ControlType.kPosition);
        Logger.recordOutput("Hood Predicted Angle", hoodAngle);
        Logger.recordOutput("Hood Angle", m_hood_encoder.getPosition());
    }

    public double getKf() {
        m_flywheel_motor.setVoltage(6);

        StopWatch stopWatch = new StopWatch();
        stopWatch.start();

        while (stopWatch.getDuration() < 5) {
            Logger.recordOutput("Shooter velocity", m_flywheel_motor.getEncoder().getVelocity());
        }

        return 6 / m_flywheel_motor.getEncoder().getVelocity();
    }
}
