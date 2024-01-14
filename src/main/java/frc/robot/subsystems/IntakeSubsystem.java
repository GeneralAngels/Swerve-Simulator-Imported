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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    Mechanism2d intake = new Mechanism2d(2, 2);
    MechanismRoot2d intake_root = intake.getRoot("root", 1, 0.07);
    MechanismLigament2d intake_opening = intake_root.append(
            new MechanismLigament2d(
                    "intake opening", 0, 180, 6, new Color8Bit(Color.kRed)
            )
    );
    CANSparkMax opener = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax roller = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    DigitalInput intake_beambreaker = new DigitalInput(0);






    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

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

    @Override
    public void periodic() {
        SmartDashboard.putData("intake", intake);
    }

    public Command get_open_command() {
        return Commands.sequence(
                new InstantCommand(() -> {opener.set(3);}, this),
                Commands.waitSeconds(0.3),
                new InstantCommand(() -> {opener.set(0);}, this)
        );
    }

    public Command get_close_command() {
        return Commands.sequence(
          new InstantCommand(() -> {opener.set(-3);},this),
          Commands.waitSeconds(0.3),
          new InstantCommand(() -> {opener.set(0);}, this)
        );
    }

    public void open() {
        var command = get_open_command();
        command.schedule();
        System.out.println("opening");
        intake_opening.setLength(0.8);
    }

    public void close() {
        var command = get_close_command();
        command.schedule();
        intake_opening.setLength(0);
    }

    public void roll() {
        roller.set(0.5);
    }

    public void stopRolling() {
        roller.set(0.0);
    }

    public void eject() {
        roller.set(-0.5);
    }

    public void hasNote(){
        /*if (intake_beambreaker.get())
            // the robot doesnt have a note (beam wasn't broken)
        else
            // the robot has a note (beam was broken)*/
    }


}

