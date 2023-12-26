package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.utils.NT_Helper;

public class FalconControllerTest {
    private static FalconControllerTest instance = null;

    public TalonFX m_motor;
    public TalonFXConfiguration m_configs;
    private RelativeEncoder m_encoder;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("FalconControllerTest");

    IntegerSubscriber motor_port = NT_Helper.getIntSubscriber(table, "Motor Port", 1);

    DoubleSubscriber kp_input = NT_Helper.getDoubleSubscriber(table, "Kp", 0);
    DoubleSubscriber ki_input = NT_Helper.getDoubleSubscriber(table, "Ki", 0);
    DoubleSubscriber kd_input = NT_Helper.getDoubleSubscriber(table, "Kd", 0);
    DoubleSubscriber kf_input = NT_Helper.getDoubleSubscriber(table, "Kf", 0);

    DoubleSubscriber precent_input = NT_Helper.getDoubleSubscriber(table, "Motor Precent Input", 0);
    DoubleSubscriber position_input = NT_Helper.getDoubleSubscriber(table, "Motor Position Input", 0);
    DoubleSubscriber velocity_input = NT_Helper.getDoubleSubscriber(table, "Motor Velocity Input", 0);


    DoublePublisher kf_publisher = table.getDoubleTopic("Kf Calc Output").publish();
    DoublePublisher motor_rpm = table.getDoubleTopic("Motor RPM").publish();
    DoublePublisher motor_position = table.getDoubleTopic("Motor Position").publish();

    double kf = 0;

    public NeoControllerTest() {
        this.m_motor = new CANSparkMax((int) motor_port.get(), CANSparkMaxLowLevel.MotorType.kBrushless);
        this.m_motor.restoreFactoryDefaults();
        this.m_encoder = this.m_motor.getEncoder();
        this.m_pidController = this.m_motor.getPIDController();

        SmartDashboard.putData("Calculate Kf", new InstantCommand(this::displayKF));
        SmartDashboard.putData("Run Motor", new InstantCommand(this::runMotor));
        SmartDashboard.putData("Stop Motor", new InstantCommand(this::stopMotor));
        SmartDashboard.putData("Reset Parameters", new InstantCommand(this::resetParams));


        this.m_pidController.setD((float) kd_input.get());
        this.m_pidController.setI((float) ki_input.get());
        this.m_pidController.setP((float) kp_input.get());
        this.m_pidController.setFF((float) kf_input.get());
        this.m_motor.burnFlash();

        REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
    }

    public static NeoControllerTest getInstance() {
        if (instance == null) {
            instance = new NeoControllerTest();
        }
        return instance;
    }
    
    public void displayKF() {
        this.calculateKF();
        kf_publisher.set(this.kf);
    }

    public void resetParams() {
        m_encoder.setPosition(0);
    }

    public void calculateKF() {
        System.out.println("calc kf");
        var a = Commands.sequence(
                new RunCommand(() -> {
                    m_motor.set(0.5);
                }).withTimeout(3),
                new InstantCommand(() -> {
                    this.kf = 0.5 / m_motor.getEncoder().getVelocity();
                    m_motor.set(0);
                }));

        a.schedule();
    }

    public void renew() {
        if (this.m_motor.getDeviceId() != (int) motor_port.get()) {
            this.m_motor = new CANSparkMax((int) motor_port.get(),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
            this.m_motor.restoreFactoryDefaults();
            this.m_encoder = this.m_motor.getEncoder();
            this.m_pidController = this.m_motor.getPIDController();

            this.m_pidController.setD((float) kd_input.get());
            this.m_pidController.setI((float) ki_input.get());
            this.m_pidController.setP((float) kp_input.get());
            this.m_pidController.setFF((float) kf_input.get());
            this.m_motor.burnFlash();
        }
        if ((float) kp_input.get() != this.m_pidController.getP()) {
            this.m_pidController.setP((float) kp_input.get());
        }
        if ((float) ki_input.get() != this.m_pidController.getI()) {
            this.m_pidController.setI((float) ki_input.get());
        }
        if ((float) kd_input.get() != this.m_pidController.getD()) {
            this.m_pidController.setD((float) kd_input.get());
        }
        if ((float) kf_input.get() != this.m_pidController.getFF()) {
            this.m_pidController.setFF((float) kf_input.get());
        }
    }

    @Override
    public void _periodic() {
        renew();
        setMotorPositionAndRPM();
    }

    public void setMotorPositionAndRPM() {
        this.motor_rpm.set(this.m_encoder.getVelocity());
        this.motor_position.set(this.m_encoder.getPosition());
    }

    public void runMotor() {
        System.out.println("run");
        if ((double) this.velocity_input.get() != 0) {
            this.m_pidController.setReference((double) this.velocity_input.get(), ControlType.kVelocity);
        }
        else if ((double) this.position_input.get() != 0) {
            this.m_pidController.setReference((double) this.position_input.get(), ControlType.kPosition);
        }
        else if ((double) this.precent_input.get() != 0) {
            this.m_motor.set((double) this.precent_input.get());
        }
    }

    public void stopMotor() {
        System.out.println("stop");
        this.m_motor.set(0);
    }
}
