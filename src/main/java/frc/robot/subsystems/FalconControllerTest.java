package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;

public class FalconControllerTest extends TimeMeasurementSubsystem{
    private static FalconControllerTest instance = null;

    public TalonFX m_motor;
    public TalonFXConfiguration m_configs;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("FalconControllerTest");

    IntegerSubscriber motor_port = NT_Helper.getIntSubscriber(table, "Motor Port", 700);

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

    DoublePublisher configed = table.getDoubleTopic("configed").publish();


    double kf = 0;

    final int kUnitsPerRevolution = 2048;

    public FalconControllerTest() {
        configed.set(0);

        this.m_motor = new TalonFX((int) motor_port.get());
        this.m_configs = new TalonFXConfiguration();

        this.m_configs.Slot0.kP = (float) this.kp_input.get();
        this.m_configs.Slot0.kI = (float) this.ki_input.get();
        this.m_configs.Slot0.kD = (float) this.kd_input.get();
        this.m_configs.Slot0.kV = (float) this.kf_input.get(); // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        this.m_configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        this.m_configs.CurrentLimits.SupplyCurrentLimit = 3;
        this.m_configs.CurrentLimits.SupplyCurrentLimitEnable = true;

        this.m_configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.m_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        this.m_motor.getConfigurator().apply(this.m_configs);
        SmartDashboard.putData("Falcon - Calculate Kf", new InstantCommand(this::displayKF));
        SmartDashboard.putData("Falcon - Run Motor", new InstantCommand(this::  runMotor));
        SmartDashboard.putData("Falcon - Stop Motor", new InstantCommand(this::stopMotor));
        SmartDashboard.putData("Falcon - Reset Parameters", new InstantCommand(this::resetParams));

    }

    public static FalconControllerTest getInstance() {
        if (instance == null) {
            instance = new FalconControllerTest();
        }
        return instance;
    }

    public void displayKF() {
        this.calculateKF();
        kf_publisher.set(this.kf);
    }

    public void resetParams() {
        this.m_motor.setPosition(0);
    }

    public void calculateKF() {
        System.out.println("calc kf");
        var a = Commands.sequence(
                new RunCommand(() -> {
                    m_motor.setVoltage(5);;
                }).withTimeout(3),
                new InstantCommand(() -> {
                    this.kf = 5 / m_motor.getVelocity().getValue();
                    m_motor.set(0);
                }));

        a.schedule();
    }

    public void renew() {
        if (this.m_motor.getDeviceID() != (int) motor_port.get()) {
            this.m_motor = new TalonFX((int) this.motor_port.get());

            this.m_configs.Slot0.kP = (float) this.kp_input.get();
            this.m_configs.Slot0.kI = (float) this.ki_input.get();
            this.m_configs.Slot0.kD = (float) this.kd_input.get();
            this.m_configs.Slot0.kV = (float) this.kf_input.get(); // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

            this.m_configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

            this.m_configs.CurrentLimits.SupplyCurrentLimit = 3;
            this.m_configs.CurrentLimits.SupplyCurrentLimitEnable = true;

            this.m_configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            this.m_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            this.m_motor.getConfigurator().apply(this.m_configs);
        }
        if ((float) kp_input.get() != this.m_configs.Slot0.kP) {
            this.m_configs.Slot0.kP = (float) kp_input.get();
            this.m_motor.getConfigurator().apply(this.m_configs);
        }
        if ((float) ki_input.get() != this.m_configs.Slot0.kI) {
            this.m_configs.Slot0.kI = (float) ki_input.get();
            this.m_motor.getConfigurator().apply(this.m_configs);
        }
        if ((float) kd_input.get() != this.m_configs.Slot0.kD) {
            this.m_configs.Slot0.kD = (float) kd_input.get();
            this.m_motor.getConfigurator().apply(this.m_configs);
        }
        if ((float) kf_input.get() != this.m_configs.Slot0.kV) {
            System.out.println("\n\nconfigining kv\n\n");
            configed.set(10);
            this.m_configs.Slot0.kV = (float) kf_input.get();
            this.m_motor.getConfigurator().apply(this.m_configs);
        }
    }

    @Override
    public void _periodic() {
        renew();
        setMotorPositionAndRPM();
    }

    public void setMotorPositionAndRPM() {
        this.motor_rpm.set(this.m_motor.getVelocity().getValue());
        this.motor_position.set(this.m_motor.getPosition().getValue());
    }

    public void runMotor() {
        System.out.println("run");
        if ((double) this.velocity_input.get() != 0) {
            m_motor.setControl(new VelocityVoltage(velocity_input.get()).withSlot(0).withAcceleration(0).withEnableFOC(false));
        }
        else if ((double) this.position_input.get() != 0) {
            PositionVoltage position_request = new PositionVoltage(0).withSlot(0).withPosition(0);
            m_motor.setControl(position_request.withPosition(this.position_input.get()));
        }
        else if ((double) this.precent_input.get() != 0) {
            m_motor.set(precent_input.get());
        }
    }

    public void stopMotor() {
        System.out.println("stop");
        this.m_motor.set(0);
    }
    
}
