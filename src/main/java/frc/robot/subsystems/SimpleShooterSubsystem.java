package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

public class SimpleShooterSubsystem extends SubsystemBase {

    Mechanism2d shooter = new Mechanism2d(2,2);
    MechanismRoot2d r_shooter = shooter.getRoot("Root",1,0.07);
    MechanismLigament2d l_shooter = r_shooter.append(new MechanismLigament2d("Shooter",0,90,6, new Color8Bit(Color.kOrange)));
    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this SimpleShooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static SimpleShooterSubsystem INSTANCE = new SimpleShooterSubsystem();

    /**
     * Returns the Singleton instance of this SimpleShooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code SimpleShooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static SimpleShooterSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this SimpleShooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private SimpleShooterSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        SmartDashboard.putData(
                "shoot", new InstantCommand(this::shoot)
        );

        SmartDashboard.putData(
                "close shooter", new InstantCommand(this::stopShooting)
        );

        SmartDashboard.putData("shooter",shooter);
    }

    @Override
    public void periodic(){
    }

    public void shoot() {
        l_shooter.setLength(0.8);
    }

    public void stopShooting(){
        l_shooter.setLength(0.0);
    }

    public Command getDefaultShootingCommand() {
        return Commands.sequence(
                new InstantCommand(this::shoot, SimpleShooterSubsystem.getInstance()),
                new WaitCommand(0.4),
                new InstantCommand(this::stopShooting, SimpleShooterSubsystem.getInstance())
        );
    }
}

