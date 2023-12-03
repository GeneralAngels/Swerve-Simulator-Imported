package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;

import frc.robot.subsystems.utils.TimeMeasurementSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends TimeMeasurementSubsystem {
    public CANSparkMax m_flywheel_motor;
    private static final int flywheel_deviceId = 100;
    public SparkMaxPIDController m_flywheel_pidController;
    private RelativeEncoder m_flywheel_encoder;

    public CANSparkMax m_hood_motor;
    public SparkMaxPIDController m_hood_PidController;
    private RelativeEncoder m_hood_encoder;
    private static final int hood_deviceId = 200;


    public double desiredVelocity = 0.0;

    private static Shooter instance = null;

    public boolean isRegular = true;

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

        m_hood_PidController.setP(ShooterConstants.HOOD_P, 0);
        m_hood_PidController.setI(ShooterConstants.HOOD_I, 0);
        m_hood_PidController.setD(ShooterConstants.HOOD_D, 0);
        m_hood_PidController.setFF(ShooterConstants.HOOD_F, 0);
        m_hood_PidController.setOutputRange(-1, 1);

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
    public void _periodic() {
        // m_flywheel_pidController.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity); // We can use smart velocity if we want.

        Logger.recordOutput("shooter velocity", m_flywheel_motor.getEncoder().getVelocity());
        setHoodAngle();

        var velocity_to_set = desiredVelocity;
        if (isRegular) {
            desiredVelocity = ShooterConstants.FLYWHEEL_RPM_MAP.get(getDistanceToTarget()) / 2;
        }

        m_flywheel_pidController.setReference(desiredVelocity, ControlType.kVelocity);

    }

    public double getHoodEncoder(double distanceToTarget) {
        if (Robot.isReal()) {
            return m_hood_encoder.getPosition();
        } else {
            return ShooterConstants.HOOD_ANGLE_MAP.get(distanceToTarget);
        }
    }

    public void setDesiredVelocity(double velocity) {
        desiredVelocity = velocity;
    }

    public boolean atDesiredVelocity() {
        return Math.abs(m_flywheel_encoder.getVelocity() - desiredVelocity) <= 0.05;
    }

    public void setHoodAngle() {

        double distanceToTarget = getDistanceToTarget();
        distanceToTarget = 2.0;
        double hoodAngle = ShooterConstants.HOOD_ANGLE_MAP.get(distanceToTarget);

        Logger.recordOutput("Hood Predicted Angle", hoodAngle);
        Logger.recordOutput("Hood Angle", getHoodEncoder(distanceToTarget));
    }

    public Command getKf() {
        return Commands.sequence(
                new RunCommand(() -> {
                    m_hood_motor.set(0.5);
                }).withTimeout(3),
                new RunCommand(() -> {
                    Logger.recordOutput("NEW KF", 0.5 / m_hood_motor.getEncoder().getVelocity());
                })
        );
    }

    public double getDistanceToTarget() {
        Transform2d poseToTarget = new Transform2d(NewPoseEstimatorSubsystem.getInstance().getCurrentPose(), ShooterConstants.TARGET_APRIL);
        return Math.hypot(poseToTarget.getX(), poseToTarget.getY());
    }
}
