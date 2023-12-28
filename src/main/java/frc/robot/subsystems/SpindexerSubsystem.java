
package frc.robot.subsystems;

import com.ctre.phoenix.time.StopWatch;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;



public class SpindexerSubsystem extends SubsystemBase {

    public enum CurrentSpindexerState{
        SPINNING,
        STATIC
    }

    public enum FeederState{
        OPEN,
        CLOSED
    }
    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.
    CANSparkMax motor2 = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    DigitalInput beam_breaker = new DigitalInput(4);
    Solenoid solenoid3 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    int ballsShot = 0;
    CurrentSpindexerState spindexerState = CurrentSpindexerState.STATIC;
    FeederState feederState = FeederState.CLOSED;

    Mechanism2d spindexer = new Mechanism2d(2,2);
    Mechanism2d feeder = new Mechanism2d(2,2);
    MechanismRoot2d spindexer_root = spindexer.getRoot("Spindexer Place", 1, 0.09);

    MechanismRoot2d feeder_root = feeder.getRoot("Feeder Place", 1.1, 0.35);

    MechanismLigament2d m_spindexer = spindexer_root.append(new MechanismLigament2d("Spindexer",0.4,0,6, new Color8Bit(Color.kYellow)));
    MechanismLigament2d m_feeder = feeder_root.append(new MechanismLigament2d("Feeder",0,270,5, new Color8Bit(Color.kWhite)));



    /**
     * The Singleton instance of this SpindexerSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static SpindexerSubsystem INSTANCE = new SpindexerSubsystem();


    /**
     * Returns the Singleton instance of this SpindexerSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code SpindexerSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static SpindexerSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this SpindexerSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private SpindexerSubsystem() {
        System.out.println("putting data - Spindexer");
        SmartDashboard.putData("Spindexer System", spindexer);


        SmartDashboard.putData("Spinning Spindexer", new InstantCommand(
                () -> {
                    System.out.println("we are working");
                    this.spin();
                }
        ));
        SmartDashboard.putData("Stopping Spindexer", new InstantCommand(
                () -> {
                    System.out.println("we are working");
                    this.stopSpin();
                }
        ));

        SmartDashboard.putData("Feeder",feeder);
        SmartDashboard.putData("Open Feeder", new InstantCommand(
                () -> {
                    System.out.println("we are working");
                    this.openPiston();
                }
        ));
        SmartDashboard.putData("Close feeder", new InstantCommand(
                () -> {
                    System.out.println("we are working");
                    this.closePiston();
                }
        ));
    }

    @Override
    public void periodic()  {
        if (NewPoseEstimatorSubsystem.getInstance().getCurrentPose().getTranslation().getDistance(new Translation2d(0,0)) < 4) {
            //this.spin();
            new ShootCommand();
        }
        this.CountBalls();
    }

    public void spin() {
        spindexerState = CurrentSpindexerState.SPINNING;
        motor2.setVoltage(3.0);
        m_spindexer.setAngle(180);


    }

    public void stopSpin() {
        spindexerState = CurrentSpindexerState.STATIC;
        motor2.setVoltage(0.0);
        m_spindexer.setAngle(0);
    }

    public void openPiston() {
        feederState = FeederState.OPEN;
        solenoid3.set(true);
        m_feeder.setLength(0.1);
    }
    
    public void closePiston() {
        feederState = FeederState.CLOSED;
        solenoid3.set(false);
        m_feeder.setLength(0);

    }

    public boolean inTower(){
        return !(beam_breaker.get());
    }

    public void CountBalls(){
        if (inTower())
            ballsShot++;
    }

    public boolean shotAllBalls(){
        return ballsShot == 5;
    }

    public void restartBalls() {
        ballsShot = 0;
    }
}

