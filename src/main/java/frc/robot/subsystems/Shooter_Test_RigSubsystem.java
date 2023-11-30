package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_Test_RigSubsystem extends SubsystemBase {
    CANSparkMax flywheel_motor1;
    CANSparkMax flywheel_motor2;
    CANSparkMax roller_motor;

    GenericEntry flywheel_1_port = Shuffleboard.getTab("SHOOTER RIG").add("flywheel port 1", 1)
            .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).getEntry();
    GenericEntry flywheel_2_port = Shuffleboard.getTab("SHOOTER RIG").add("flyhweel port 2", 2)
            .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).getEntry();
    GenericEntry rollers_port = Shuffleboard.getTab("SHOOTER RIG").add("hood port 1", 3)
            .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).getEntry();

    GenericEntry flywheel_1_percent = Shuffleboard.getTab("SHOOTER RIG").add("flywheel 1 speed", 0.1)
            .withWidget(BuiltInWidgets.kNumberSlider).withSize(3, 2).getEntry();
    GenericEntry flywheel_2_percent = Shuffleboard.getTab("SHOOTER RIG").add("flywheel 2 speed", -0.1)
            .withWidget(BuiltInWidgets.kNumberSlider).withSize(3, 2).getEntry();
    GenericEntry roller_percent = Shuffleboard.getTab("SHOOTER RIG").add("hood  speed", 0.1)
            .withWidget(BuiltInWidgets.kNumberSlider).withSize(3, 2).getEntry();

    GenericEntry flywheel_1_RPM = Shuffleboard.getTab("SHOOTER RIG").add("flywheel 1 RPM", 500)
            .withWidget(BuiltInWidgets.kTextView).withSize(2, 2).getEntry();
    GenericEntry flywheel_2_RPM = Shuffleboard.getTab("SHOOTER RIG").add("flywheel 2 RPM", -500)
            .withWidget(BuiltInWidgets.kTextView).withSize(2, 2).getEntry();
    GenericEntry roller_RPM = Shuffleboard.getTab("SHOOTER RIG").add("rollers RPM", 500)
            .withWidget(BuiltInWidgets.kTextView).withSize(2, 2).getEntry();

    public SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.05); // in percent per sec^2

    double Kf = 0.002;
    double Kp = 0.0015;

    public Shooter_Test_RigSubsystem() {
        this.flywheel_motor1 = new CANSparkMax((int) flywheel_1_port.getInteger(1),
                CANSparkMaxLowLevel.MotorType.kBrushless);
        this.flywheel_motor2 = new CANSparkMax((int) flywheel_2_port.getInteger(2),
                CANSparkMaxLowLevel.MotorType.kBrushless);
        this.roller_motor = new CANSparkMax((int) rollers_port.getInteger(3), CANSparkMaxLowLevel.MotorType.kBrushless);

        slewRateLimiter.reset(0);
    }

    public void renew() {
        if (this.flywheel_motor1.getDeviceId() != (int) flywheel_1_port.getInteger(flywheel_motor1.getDeviceId())) {
            this.flywheel_motor1 = new CANSparkMax((int) flywheel_1_port.getInteger(flywheel_motor1.getDeviceId()),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }

        if (this.flywheel_motor2.getDeviceId() != (int) flywheel_2_port.getInteger(flywheel_motor2.getDeviceId())) {
            this.flywheel_motor2 = new CANSparkMax((int) flywheel_2_port.getInteger(flywheel_motor2.getDeviceId()),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }

        if (this.roller_motor.getDeviceId() != (int) rollers_port.getInteger(roller_motor.getDeviceId())) {
            this.roller_motor = new CANSparkMax((int) rollers_port.getInteger(roller_motor.getDeviceId()),
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        }
    }

    public void teleopPeriodicPercent() {
        this.flywheel_motor1.set(flywheel_1_percent.getDouble(0.1));
        this.flywheel_motor1.set(flywheel_2_percent.getDouble(-0.1));

        var target_percent = roller_percent.getDouble(0.1);
        SmartDashboard.putNumber("target percent", target_percent);

        var rated_percent = slewRateLimiter.calculate(target_percent);

        SmartDashboard.putNumber("rated percent", rated_percent);
        this.roller_motor.set(rated_percent);

        flywheel_1_RPM.setDouble(flywheel_motor1.getEncoder().getVelocity());
        flywheel_2_RPM.setDouble(flywheel_motor2.getEncoder().getVelocity());
        roller_RPM.setDouble(roller_motor.getEncoder().getVelocity());
    }

    @Override
    public void periodic() {
        var target_percent = roller_percent.getDouble(0.1);
        SmartDashboard.putNumber("target percent", target_percent);

        SmartDashboard.putNumber("output", SmartDashboard.getNumber("input", 1));
    }

    public void RPM_periodic() {
        double target_1_rpm = flywheel_1_RPM.getDouble(500);
        flywheel_motor1.setVoltage(Kf * target_1_rpm +
                Kp * (target_1_rpm - flywheel_motor1.getEncoder().getVelocity()));

        double target_2_rpm = flywheel_2_RPM.getDouble(-500);
        flywheel_motor2
                .setVoltage(Kf * target_2_rpm +
                        Kp * (target_2_rpm - flywheel_motor2.getEncoder().getVelocity()));

        double target_roller_RPM = roller_RPM.getDouble(500);
        roller_motor.setVoltage(
                Kf * target_roller_RPM +
                        Kp * (target_roller_RPM - roller_motor.getEncoder().getVelocity()));

        Logger.recordOutput("rpm1", flywheel_motor1.getEncoder().getVelocity());
        Logger.recordOutput("rpm2", flywheel_motor2.getEncoder().getVelocity());
        Logger.recordOutput("rollers rpm", roller_motor.getEncoder().getVelocity());
    }
}
