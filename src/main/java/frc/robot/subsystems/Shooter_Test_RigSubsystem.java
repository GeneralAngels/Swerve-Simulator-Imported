package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.utils.NT_Helper;
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Shooter_Test_RigSubsystem extends TimeMeasurementSubsystem {
    CANSparkMax flywheel_motor1;
    CANSparkMax roller_motor;
    Encoder m_hood_encoder;
    DutyCycleEncoder m_hoodDutyCycleEncoder;

    SparkMaxPIDController roller_PidController;
    SparkMaxPIDController flywheel_PidController;


    NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterRig");

    IntegerSubscriber flywheel_1_port = NT_Helper.getIntSubscriber(table, "flywheel 1 port", 4);
    IntegerSubscriber rollers_port = NT_Helper.getIntSubscriber(table, "flywheel 3 port", 5);

    DoubleSubscriber flywheel_1_percent = NT_Helper.getDoubleSubscriber(table, "flywheel 1 percent", 0.0);
    DoubleSubscriber rollers_percent = NT_Helper.getDoubleSubscriber(table, "rollers percent", 0.0);
    DoubleSubscriber custom = NT_Helper.getDoubleSubscriber(table, "custom", 0.0);

    DoublePublisher flywheel_1_RPM = table.getDoubleTopic("flywheel1 RPM").publish();
    DoublePublisher rollers_RPM = table.getDoubleTopic("rollers RPM").publish();
    DoublePublisher hood_encoder = table.getDoubleTopic("Hood Encoder:").publish();

    DoubleSubscriber distance = NT_Helper.getDoubleSubscriber(table, "distance from target", 1);

    DoubleSubscriber kp_entry = NT_Helper.getDoubleSubscriber(table, "kp", 0.00007);

    // NOTICE: In elastic only text display input works!
    // Let's check on robot also.

    public SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.15); // in percent per sec^2

    double Kf = 0.0001620370333740905;
    double Kp = 0.00007;
    double Ki = 0.000000006;

    public Shooter_Test_RigSubsystem() {
        this.flywheel_motor1 = new CANSparkMax((int) flywheel_1_port.get(),
                CANSparkMaxLowLevel.MotorType.kBrushless);
        this.roller_motor = new CANSparkMax((int) rollers_port.get(), CANSparkMaxLowLevel.MotorType.kBrushless);
        //this.m_hood_encoder = new Encoder(7, 5);
        this.m_hoodDutyCycleEncoder = new DutyCycleEncoder(7);

        this.roller_PidController = roller_motor.getPIDController();
        this.roller_PidController.setP(0);
        this.roller_PidController.setI(Ki);
        this.roller_PidController.setD(0);
        this.roller_PidController.setFF(Kf);
        this.roller_motor.burnFlash();

        this.flywheel_PidController = flywheel_motor1.getPIDController();


        slewRateLimiter.reset(0);
    }

    public void renew() {
        if (this.flywheel_motor1.getDeviceId() != (int) flywheel_1_port.get()) {
            this.flywheel_motor1 = new CANSparkMax((int) flywheel_1_port.get(),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }

        if (this.roller_motor.getDeviceId() != (int) rollers_port.get()) {
            this.roller_motor = new CANSparkMax((int) rollers_port.get(),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }
    }

    public void teleopPeriodicPercent() {
        roller_PidController.setP(kp_entry.get());
        this.flywheel_motor1.set(flywheel_1_percent.get());

        var target_percent = rollers_percent.get();
        //var rated_percent = slewRateLimiter.calculate(target_percent);
        
        this.roller_motor.set(target_percent);
        //this.roller_PidController.setReference(ShooterConstants.FLYWHEEL_RPM_MAP.get(distance.get()) * 2, ControlType.kVelocity);

        flywheel_1_RPM.set(flywheel_motor1.getEncoder().getVelocity());
        rollers_RPM.set(roller_motor.getEncoder().getVelocity());
        hood_encoder.set(this.m_hoodDutyCycleEncoder.getAbsolutePosition());

        Logger.recordOutput("Hood Encoder get()", this.m_hoodDutyCycleEncoder.get());
        Logger.recordOutput("Hood Encoder getAbsolutePosition()", this.m_hoodDutyCycleEncoder.getAbsolutePosition());


        Logger.recordOutput("Flywheel rpm", flywheel_motor1.getEncoder().getVelocity());
        Logger.recordOutput("Roller rpm", roller_motor.getEncoder().getVelocity());
        }

    public Command getKf() {
        return Commands.sequence(
                new RunCommand(() -> {
                    roller_motor.set(0.5);
                }).withTimeout(3),
                new RunCommand(() -> {
                    Logger.recordOutput("NEW KF", 0.5 / roller_motor.getEncoder().getVelocity());
                })
        );
    }

    @Override
    public void _periodic() {
        //teleopPeriodicPercent();
    }

    public void RPM_periodic() {
        double target_1_rpm = flywheel_1_percent.get();
        flywheel_motor1.setVoltage(Kf * target_1_rpm +
                Kp * (target_1_rpm - flywheel_motor1.getEncoder().getVelocity()));

        double target_roller_RPM = rollers_percent.get();
        roller_motor.setVoltage(
                Kf * target_roller_RPM +
                        Kp * (target_roller_RPM - roller_motor.getEncoder().getVelocity()));

        Logger.recordOutput("rpm1", flywheel_motor1.getEncoder().getVelocity());
        Logger.recordOutput("rollers rpm", roller_motor.getEncoder().getVelocity());
    }
}
