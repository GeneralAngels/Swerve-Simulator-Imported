package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.utils.NT_Helper;
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;

public class NeoControllerTest extends TimeMeasurementSubsystem {
    private static NeoControllerTest instance = null;

    public CANSparkMax m_motor;
    public SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("NeoControllerTest");

    IntegerSubscriber motor_port = NT_Helper.getIntSubscriber(table, "Motor Port", 1);

    DoubleSubscriber kp_input = NT_Helper.getDoubleSubscriber(table, "Kp", 0);
    DoubleSubscriber ki_input = NT_Helper.getDoubleSubscriber(table, "Ki", 0);
    DoubleSubscriber kd_input = NT_Helper.getDoubleSubscriber(table, "Kd", 0);
    DoubleSubscriber kf_input = NT_Helper.getDoubleSubscriber(table, "Kf", 0);

    DoubleSubscriber precent_input = NT_Helper.getDoubleSubscriber(table, "Motor Precent Input", 0);
    DoubleSubscriber position_input = NT_Helper.getDoubleSubscriber(table, "Motor Position Input", 0);
    DoubleSubscriber velocity_input = NT_Helper.getDoubleSubscriber(table, "Motor Velocity Input", 0);


    DoublePublisher kf_publisher = table.getDoubleTopic("Kf").publish();
    DoublePublisher motor_rpm = table.getDoubleTopic("Motor RPM").publish();
    DoublePublisher motor_position = table.getDoubleTopic("Motor Position").publish();

    double kf = 0;

    public NeoControllerTest() {
        this.m_motor = new CANSparkMax((int) motor_port.get(), CANSparkMaxLowLevel.MotorType.kBrushless);
        this.m_motor.restoreFactoryDefaults();
        this.m_encoder = this.m_motor.getEncoder();
        this.m_pidController = this.m_motor.getPIDController();

        this.m_pidController.setD(kd_input.get());
        this.m_pidController.setI(ki_input.get());
        this.m_pidController.setP(kp_input.get());
        this.m_pidController.setFF(kf_input.get());
        this.m_motor.burnFlash();
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

    public void calculateKF() {
        var a = Commands.sequence(
                new RunCommand(() -> {
                    m_motor.set(0.5);
                }).withTimeout(3),
                new InstantCommand(() -> {
                    this.kf = 0.5 / m_motor.getEncoder().getVelocity();
                }));

        a.schedule();
    }
}
