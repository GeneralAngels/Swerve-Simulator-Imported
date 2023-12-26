package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.*;

public class IntakeSubsystem extends SubsystemBase {

    public enum CurrentIntakeState {
        OPEN,
        TAKING,
        EJECTING,
        CLOSED
    }

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    CANSparkMax motor1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

    Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    Mechanism2d intake = new Mechanism2d(2,2);
    MechanismRoot2d root1 = intake.getRoot("Intake", 1, 0.07);
//    MechanismRoot2d root2 = intake.getRoot("Intake2", 1, 0.07);
    MechanismLigament2d piston1 = root1.append(new MechanismLigament2d("Solenoid 1",0, 0, 6, new Color8Bit(Color.kPurple)));
//    MechanismLigament2d piston2 = root2.append(new MechanismLigament2d("Solenoid 2",0,0));
    CurrentIntakeState currentState = CurrentIntakeState.CLOSED;


    /**
     * The Singleton instance of this IntakeSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code IntakeSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private IntakeSubsystem() {
        System.out.println("putting data- intake");
        SmartDashboard.putData("Intake System", intake);

        SmartDashboard.putData("opening intake", new InstantCommand(
                () -> {
                    System.out.println("we are working");
                    this.open();
                }
        ));
        SmartDashboard.putData("closing intake", new InstantCommand(
                () -> {
                    System.out.println("we are working");
                    this.close();
                }
        ));
    }

    @Override
    public void periodic()  {

    }

    public void take() {
        // TODO: activate motor in positive direction.
        currentState = CurrentIntakeState.TAKING;
        motor1.set(0.7);
    }

    public void stopTaking() {
        motor1.setVoltage(0.0);
    }

    public void eject() {
        currentState = CurrentIntakeState.EJECTING;
        motor1.set(-0.7);

    }

    public void open() {
        currentState = CurrentIntakeState.OPEN;
        solenoid1.set(true);
        solenoid2.set(true);
        piston1.setLength(0.7);
//        piston2.setLength(1);

    }
    public void close() {
        currentState = CurrentIntakeState.CLOSED;
        solenoid1.set(false);
        solenoid2.set(false);
        piston1.setLength(0);
//        piston2.setLength(0);

    }

}

