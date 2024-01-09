package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.utils.NT_Helper;
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends TimeMeasurementSubsystem {
    public CANSparkMax m_flywheel_motor;
    private static final int flywheel_deviceId = 400;
    public SparkMaxPIDController m_flywheel_pidController;
    private RelativeEncoder m_flywheel_encoder;

    public CANSparkMax m_hood_motor;
    public SparkMaxPIDController m_hood_PidController;
    private RelativeEncoder m_hood_encoder;
    private static final int hood_deviceId = 300;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter");

    DoubleSubscriber distance = NT_Helper.getDoubleSubscriber(table, "distance to target", 1);

    DoublePublisher flywheel_rpm = table.getDoubleTopic("Fly Wheel RPM").publish();
    DoublePublisher hood_rotation = table.getDoubleTopic("Hood Rotation").publish();
    DoublePublisher hood_in_angle = table.getDoubleTopic("Hood Angle").publish();

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
        Logger.recordOutput("motor voltage", m_flywheel_motor.getAppliedOutput());
        setHoodAngle();

        if (isRegular) {
            desiredVelocity = ShooterConstants.FLYWHEEL_RPM_MAP.get(getDistanceToTarget()) / 2;
        }
        Logger.recordOutput("Wanted RPM", desiredVelocity);

        m_flywheel_pidController.setReference(desiredVelocity, ControlType.kVelocity);

        flywheel_rpm.set(m_flywheel_encoder.getVelocity());
        hood_rotation.set(m_hood_encoder.getPosition());
        hood_in_angle.set(m_hood_encoder.getPosition() * 360);
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
        double hoodAngle = ShooterConstants.HOOD_ANGLE_MAP.get(distanceToTarget);

        m_hood_PidController.setReference(hoodAngle / 360, ControlType.kPosition);

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
        return this.distance.get();
        //Transform2d poseToTarget = new Transform2d(NewPoseEstimatorSubsystem.getInstance().getCurrentPose(), ShooterConstants.TARGET_APRIL);
        //return Math.hypot(poseToTarget.getX(), poseToTarget.getY());
    }
}
