package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class CANSparkMaxTest extends SubsystemBase {
    private static final int deviceID = 3;
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public CANSparkMaxTest() 
    {
        m_motor = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);

        /**
         * The restoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_motor.restoreFactoryDefaults();

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = m_motor.getPIDController();

        // Encoder object created to display position values
        m_encoder = m_motor.getEncoder();

        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        //m_flywheel_pidController.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity); // We can use smart velocity if we want.
        //Logger.recordOutput("shooter velocity", m_flywheel_motor.getEncoder().getVelocity());
        // setHoodAngle();

        m_pidController.setReference(3, ControlType.kPosition);
        System.out.println(m_encoder.getPosition());
    }
}
