package frc.robot.subsystems;

import com.ctre.phoenix.time.StopWatch;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    public CANSparkMax m_flywheel_motor;
    private static final int flywheel_deviceId = 4;
    public SparkMaxPIDController m_flywheel_pidController;
    private RelativeEncoder m_flywheel_encoder;

    public CANSparkMax m_hood_motor;
    public SparkMaxPIDController m_hood_PidController;
    private RelativeEncoder m_hood_encoder;
    private static final int hood_deviceId = 200;
    public double desiredVelocity = 0.0;

    private static Shooter instance = null;

    public boolean isRegular = true;

   Mechanism2d shooter = new Mechanism2d(2,2);
    MechanismRoot2d shooter_root = shooter.getRoot("Tower Place", 0.9, 0.07);
    MechanismLigament2d m_shooter = shooter_root.append(new MechanismLigament2d("Tower",0.7, 90, 6, new Color8Bit(Color.kBlue)));
    MechanismLigament2d m_hood = m_shooter.append(new MechanismLigament2d("Hood",0.45, hoodAngle() * -1, 5, new Color8Bit(Color.kBlue)));

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

        System.out.println("putting data - Shooter");
        System.out.println(hoodAngle());
        SmartDashboard.putData("Shooter System", shooter);
    }

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    @Override
    public void periodic() {
        // m_flywheel_pidController.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity); // We can use smart velocity if we want.
        m_flywheel_pidController.setReference(3000, ControlType.kVelocity);
        Logger.recordOutput("shooter velocity", m_flywheel_motor.getEncoder().getVelocity());
        Logger.recordOutput("motor voltage", m_flywheel_motor.getAppliedOutput());
        setHoodAngle();

        if (isRegular) {
            desiredVelocity = ShooterConstants.FLYWHEEL_RPM_MAP.get(getDistanceToTarget()) / 2;
        }
        Logger.recordOutput("Wanted RPM", desiredVelocity);

    //    m_flywheel_pidController.setReference(desiredVelocity, ControlType.kVelocity);
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

        Logger.recordOutput("Hood Predicted Angle", hoodAngle);
        Logger.recordOutput("Hood Angle", getHoodEncoder(distanceToTarget));
    }

    public double getKf() {
        m_flywheel_motor.setVoltage(6);

        StopWatch stopWatch = new StopWatch();
        stopWatch.start();

        while (stopWatch.getDuration() < 5) {
            Logger.recordOutput("Shooter KF Vel", m_flywheel_motor.getEncoder().getVelocity());
        }
        Logger.recordOutput("Kf_glywheel", 6 / m_flywheel_motor.getEncoder().getVelocity());
        return 6 / m_flywheel_motor.getEncoder().getVelocity();
    }

    public double getDistanceToTarget() {
        Transform2d poseToTarget = new Transform2d(NewPoseEstimatorSubsystem.getInstance().getCurrentPose(), ShooterConstants.TARGET_APRIL);
        return Math.hypot(poseToTarget.getX(), poseToTarget.getY());
    }

    public double hoodAngle() {
        return ShooterConstants.HOOD_ANGLE_MAP.get(getDistanceToTarget());
    }
}
