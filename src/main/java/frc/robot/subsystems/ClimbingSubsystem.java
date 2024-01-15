package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {

    CANSparkMax opener = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax closer = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    DigitalInput opener_limitswitch = new DigitalInput(4);
    DigitalInput closer_limitswitch = new DigitalInput(5);
    public enum ClimbingState {
        CLIMBING,
        STATIC
    }
    ClimbingState climbingState = ClimbingState.STATIC;


    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this IntakeSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ClimbingSubsystem INSTANCE = new ClimbingSubsystem();

    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code IntakeSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ClimbingSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ClimbingSubsystem() {

    }

    @Override
    public void periodic() {
    }

    public void open() {
        Commands.sequence(
                new InstantCommand(() -> {opener.set(0.3);}),
                Commands.waitUntil(() -> {return opener_limitswitch.get();}),
                new InstantCommand(() -> {opener.set(0);})
        );
        climbingState = ClimbingState.CLIMBING;
    }

    public void close() {
        Commands.sequence(
                new InstantCommand(() -> {closer.set(-0.3);}),
                Commands.waitUntil(() -> {return closer_limitswitch.get();}),
                new InstantCommand(() -> {closer.set(0);})
        );
        climbingState = ClimbingState.STATIC;
    }

}

