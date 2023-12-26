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

    public FalconControllerTest() {

        this.m_motor = new TalonFX((int) motor_port.get());
        this.m_configs = new TalonFXConfiguration();
        //SmartDashboard.putData("Calculate Kf", new InstantCommand(this::displayKF));
        //SmartDashboard.putData("Run Motor", new InstantCommand(this::runMotor));
        //SmartDashboard.putData("Stop Motor", new InstantCommand(this::stopMotor));
        //SmartDashboard.putData("Reset Parameters", new InstantCommand(this::resetParams));

    }

    public static FalconControllerTest getInstance() {
        if (instance == null) {
            instance = new FalconControllerTest();
        }
        return instance;
    }
    
}
