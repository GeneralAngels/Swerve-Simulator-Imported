package frc.robot.subsystems;


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


    Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 23);

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

    }

    public void take() {
        // TODO: activate motor in positive direction.
        currentState = CurrentIntakeState.TAKING;
        motor1.set(1.0);
    }

    public void eject() {
        currentState = CurrentIntakeState.EJECTING;
        motor1.set(-1.0);
    }

    public void open() {
        currentState = CurrentIntakeState.OPEN;
        solenoid.set(true);
    }
    public void close() {
        currentState = CurrentIntakeState.CLOSED;
        solenoid.set(false);
    }

    public Command getTakeCommand() {
        return Commands.sequence(
                new InstantCommand(this::open),
                new WaitCommand(0.1),
                new RunCommand(this::take),
                /*new WaitCommand(3.0),
                new InstantCommand(this;;close())*/

        )
    }

    public intake {

    }
}

