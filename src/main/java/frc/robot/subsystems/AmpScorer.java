package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpScorer extends SubsystemBase {

    Mechanism2d amp = new Mechanism2d(2,2);
    MechanismRoot2d r_amp = amp.getRoot("Root",0.5,0.5);
    MechanismLigament2d l_amp = r_amp.append(new MechanismLigament2d("Amp",0,70,6, new Color8Bit(Color.kBlue)));
    CANSparkMax amp_opener = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax amp_roller = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);

    DigitalInput top_limitswitch = new DigitalInput(10);
    DigitalInput bottom_limitswitch = new DigitalInput(18);

    public enum AmpOpenerState{
        OPEN,
        CLOSED
    }

    public enum AmpRollerState{
        ROLLING,
        STATIC
    }

    AmpOpenerState ampOpenerState = AmpOpenerState.CLOSED;
    AmpRollerState ampRollerState = AmpRollerState.STATIC;

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

        SmartDashboard.putData("ampScorer", amp);
        SmartDashboard.putData("open amp", new InstantCommand(() -> {l_amp.setLength(0.5);}));
        SmartDashboard.putData("close amp", new InstantCommand(() -> {l_amp.setLength(0);}));
    }

    @Override
    public void periodic() {

    }

    public void openAmp(){
        Commands.sequence(
                new InstantCommand(() -> {amp_opener.set(0.3);}),
                Commands.waitUntil(() -> {return top_limitswitch.get();}),
                new InstantCommand(() -> {amp_opener.set(0);})
        );
        ampOpenerState = AmpOpenerState.OPEN;
        l_amp.setLength(0.4);
    }

    public void closeAmp() {
        Commands.sequence(
                new InstantCommand(() -> {amp_opener.set(-0.3);}),
                Commands.waitUntil(() -> {return bottom_limitswitch.get();}),
                new InstantCommand(() -> {amp_opener.set(0);})
        );
        ampOpenerState = AmpOpenerState.CLOSED;
        l_amp.setLength(0);
    }

    public void rollAmp(){
        amp_roller.set(0.7);
        ampRollerState = AmpRollerState.ROLLING;
    }

    public void stopRollAmp(){
        amp_roller.set(0);
        ampRollerState = AmpRollerState.STATIC;
    }
}

