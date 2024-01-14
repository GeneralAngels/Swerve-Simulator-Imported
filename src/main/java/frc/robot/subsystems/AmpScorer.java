package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpScorer extends SubsystemBase {

    CANSparkMax amp_opener = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax amp_roller = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);

    DigitalInput top_limitswitch = new DigitalInput(1);
    DigitalInput bottom_limitswitch = new DigitalInput(2);

    public enum AmpOpenerState{
        OPEN,
        CLOSED
    }

    public enum AmpRollerState{
        ROLLING,
        STATIC
    }

    AmpOpenerState AmpOpener = AmpOpenerState.CLOSED;
    AmpRollerState AmpRoller = AmpRollerState.STATIC;

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this IntakeSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static AmpScorer INSTANCE = new AmpScorer();

    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code IntakeSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static AmpScorer getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private AmpScorer() {

    }

    @Override
    public void periodic() {
    }

    public void openAmp(){
        if (!top_limitswitch.get())
            amp_opener.set(3);
        else
            amp_opener.set(0);
        AmpOpenerState AmpOpener = AmpOpenerState.OPEN;
    }

    public void closeAmp(){
        if (!bottom_limitswitch.get())
            amp_opener.set(-3);
        else
            amp_opener.set(0);
        AmpOpenerState AmpOpener = AmpOpenerState.CLOSED;
    }

    public void rollAmp(){
        amp_roller.set(0.7);
        AmpRollerState AmpRoller = AmpRollerState.ROLLING;
    }

    public void stopRollAmp(){
        amp_roller.set(0);
        AmpRollerState AmpRoller = AmpRollerState.STATIC;
    }
}

