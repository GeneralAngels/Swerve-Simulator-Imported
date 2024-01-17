package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

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
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;

public class NeoControllerTest extends TimeMeasurementSubsystem {
    private static NeoControllerTest instance = null;

    public CANSparkMax m_motor;
    public SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    NetworkTable table;

    IntegerSubscriber motor_port;

    DoubleSubscriber kp_input;
    DoubleSubscriber ki_input;
    DoubleSubscriber kd_input;
    DoubleSubscriber kf_input;

    DoubleSubscriber precent_input;
    DoubleSubscriber position_input;
    DoubleSubscriber velocity_input;

    DoublePublisher kf_publisher;
    DoublePublisher motor_rpm;
    DoublePublisher motor_position;

    double kf = 0;

    String adder;

    public NeoControllerTest(String adder) {
        this.adder = adder;
        table = NetworkTableInstance.getDefault().getTable("NeoControllerTest" + adder);

        motor_port = NT_Helper.getIntSubscriber(table, "Motor Port" + adder, 200);

        kp_input = NT_Helper.getDoubleSubscriber(table, "Kp", 0);
        ki_input = NT_Helper.getDoubleSubscriber(table, "Ki", 0);
        kd_input = NT_Helper.getDoubleSubscriber(table, "Kd", 0);
        kf_input = NT_Helper.getDoubleSubscriber(table, "Kf", 0);

        precent_input = NT_Helper.getDoubleSubscriber(table, "Motor Precent Input" + adder, 0);
        position_input = NT_Helper.getDoubleSubscriber(table, "Motor Position Input" + adder, 0);
        velocity_input = NT_Helper.getDoubleSubscriber(table, "Motor Velocity Input" + adder, 0);


        kf_publisher = table.getDoubleTopic("Kf Calc Output" + adder).publish();
        motor_rpm = table.getDoubleTopic("Motor RPM" + adder).publish();
        motor_position = table.getDoubleTopic("Motor Position" + adder).publish();

        try {
        this.m_motor = new CANSparkMax((int) motor_port.get(), CANSparkMaxLowLevel.MotorType.kBrushless);
        this.m_motor.restoreFactoryDefaults();
        this.m_encoder = this.m_motor.getEncoder();
        this.m_pidController = this.m_motor.getPIDController();

        SmartDashboard.putData("Neo - Calculate Kf" + adder, new InstantCommand(this::displayKF));
        SmartDashboard.putData("Neo - Run Motor" + adder, new InstantCommand(this::runMotor));
        SmartDashboard.putData("Neo - Stop Motor" + adder, new InstantCommand(this::stopMotor));
        SmartDashboard.putData("Neo - Reset Parameters" + adder, new InstantCommand(this::resetParams));


        this.m_pidController.setD((float) kd_input.get());
        this.m_pidController.setI((float) ki_input.get());
        this.m_pidController.setP((float) kp_input.get());
        this.m_pidController.setFF((float) kf_input.get());
        this.m_motor.burnFlash();

        REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
        }
        catch (Exception e) {
            System.out.println(e);
        }
        
    }

    public static NeoControllerTest getInstance() {
        if (instance == null) {
            instance = new NeoControllerTest("");
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
        if (this.m_motor == null) {
            this.m_motor = new CANSparkMax((int) motor_port.get(), CANSparkMaxLowLevel.MotorType.kBrushless);
            this.m_motor.restoreFactoryDefaults();
            this.m_encoder = this.m_motor.getEncoder();
            this.m_pidController = this.m_motor.getPIDController();

            SmartDashboard.putData("Neo - Calculate Kf" + adder, new InstantCommand(this::displayKF));
            SmartDashboard.putData("Neo - Run Motor" + adder, new InstantCommand(this::runMotor));
            SmartDashboard.putData("Neo - Stop Motor" + adder, new InstantCommand(this::stopMotor));
            SmartDashboard.putData("Neo - Reset Parameters" + adder, new InstantCommand(this::resetParams));


            this.m_pidController.setD((float) kd_input.get());
            this.m_pidController.setI((float) ki_input.get());
            this.m_pidController.setP((float) kp_input.get());
            this.m_pidController.setFF((float) kf_input.get());
            this.m_motor.burnFlash();
        }

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

            this.m_motor.set(0.3);
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
        try {
            renew();
            setMotorPositionAndRPM();
            Logger.recordOutput("OutputTest/" + this.adder + "/Voltage", this.m_motor.getAppliedOutput());
            Logger.recordOutput("OutputTest/" + this.adder + "/Current", this.m_motor.getOutputCurrent());
        }
        catch (Exception e) {
            System.out.println(e);
        }
        
    }

    public void setMotorPositionAndRPM() {
        this.motor_rpm.set(this.m_encoder.getVelocity());
        this.motor_position.set(this.m_encoder.getPosition());
    }

    public void runMotor() {
        System.out.println("run");
        if ((double) this.velocity_input.get() != 0) {
            this.m_pidController.setReference((double) this.velocity_input.get(), ControlType.kVelocity);
        } else if ((double) this.position_input.get() != 0) {
            this.m_pidController.setReference((double) this.position_input.get(), ControlType.kPosition);
        } else if ((double) this.precent_input.get() != 0) {
            this.m_motor.set((double) this.precent_input.get());
        }
    }

    public void stopMotor() {
        System.out.println("stop");
        this.m_motor.set(0);
    }
}
