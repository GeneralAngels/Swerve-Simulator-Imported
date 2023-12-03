package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.utils.NT_Helper;
import frc.robot.subsystems.utils.TimeMeasurementSubsystem;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Shooter_Test_RigSubsystem extends TimeMeasurementSubsystem {
    CANSparkMax flywheel_motor1;
    CANSparkMax flywheel_motor2;
    CANSparkMax roller_motor;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterRig");

    IntegerSubscriber flywheel_1_port = NT_Helper.getIntSubscriber(table, "flywheel 1 port", 1);
    IntegerSubscriber flywheel_2_port = NT_Helper.getIntSubscriber(table, "flywheel 2 port", 2);
    IntegerSubscriber rollers_port = NT_Helper.getIntSubscriber(table, "flywheel 3 port", 3);

    DoubleSubscriber flywheel_1_percent = NT_Helper.getDoubleSubscriber(table, "flywheel 1 percent", 0.0);
    DoubleSubscriber flywheel_2_percent = NT_Helper.getDoubleSubscriber(table, "flywheel 2 percent", 0.0);
    DoubleSubscriber rollers_percent = NT_Helper.getDoubleSubscriber(table, "rollers percent", 0.0);
    DoubleSubscriber custom = NT_Helper.getDoubleSubscriber(table, "custom", 0.0);

    DoublePublisher flywheel_1_RPM = table.getDoubleTopic("flywheel1 RPM").publish();
    DoublePublisher flywheel_2_RPM = table.getDoubleTopic("flywheel2 RPM").publish();
    DoublePublisher rollers_RPM = table.getDoubleTopic("rollers RPM").publish();

    // NOTICE: In elastic only text display input works!
    // Let's check on robot also.

    public SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.05); // in percent per sec^2

    double Kf = 0.002;
    double Kp = 0.0015;

    public Shooter_Test_RigSubsystem() {
        this.flywheel_motor1 = new CANSparkMax((int) flywheel_1_port.get(),
                CANSparkMaxLowLevel.MotorType.kBrushless);
        this.flywheel_motor2 = new CANSparkMax((int) flywheel_2_port.get(),
                CANSparkMaxLowLevel.MotorType.kBrushless);
        this.roller_motor = new CANSparkMax((int) rollers_port.get(), CANSparkMaxLowLevel.MotorType.kBrushless);

        slewRateLimiter.reset(0);
    }

    public void renew() {
        if (this.flywheel_motor1.getDeviceId() != (int) flywheel_1_port.get()) {
            this.flywheel_motor1 = new CANSparkMax((int) flywheel_1_port.get(),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }

        if (this.flywheel_motor2.getDeviceId() != (int) flywheel_2_port.get()) {
            this.flywheel_motor2 = new CANSparkMax((int) flywheel_2_port.get(),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }

        if (this.roller_motor.getDeviceId() != (int) rollers_port.get()) {
            this.roller_motor = new CANSparkMax((int) rollers_port.get(),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }
    }

    public void teleopPeriodicPercent() {
        this.flywheel_motor1.set(flywheel_1_percent.get());
        this.flywheel_motor1.set(flywheel_2_percent.get());

        var target_percent = rollers_percent.get();
        var rated_percent = slewRateLimiter.calculate(target_percent);
        
        this.roller_motor.set(rated_percent);

        flywheel_1_RPM.set(flywheel_motor1.getEncoder().getVelocity());
        flywheel_2_RPM.set(flywheel_motor2.getEncoder().getVelocity());
        rollers_RPM.set(roller_motor.getEncoder().getVelocity());
    }

    @Override
    public void _periodic() {
        teleopPeriodicPercent();
    }

    public void RPM_periodic() {
        double target_1_rpm = flywheel_1_percent.get();
        flywheel_motor1.setVoltage(Kf * target_1_rpm +
                Kp * (target_1_rpm - flywheel_motor1.getEncoder().getVelocity()));

        double target_2_rpm = flywheel_2_percent.get();
        flywheel_motor2
                .setVoltage(Kf * target_2_rpm +
                        Kp * (target_2_rpm - flywheel_motor2.getEncoder().getVelocity()));

        double target_roller_RPM = rollers_percent.get();
        roller_motor.setVoltage(
                Kf * target_roller_RPM +
                        Kp * (target_roller_RPM - roller_motor.getEncoder().getVelocity()));

        Logger.recordOutput("rpm1", flywheel_motor1.getEncoder().getVelocity());
        Logger.recordOutput("rpm2", flywheel_motor2.getEncoder().getVelocity());
        Logger.recordOutput("rollers rpm", roller_motor.getEncoder().getVelocity());
    }
}
