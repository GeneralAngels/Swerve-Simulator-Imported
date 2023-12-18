
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;



public class SpindexerSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.
    CANSparkMax motor2 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    DigitalInput beam_breaker = new DigitalInput(5);
    Solenoid solenoid3 = new Solenoid(PneumaticsModuleType.CTREPCM, 25);
    int ballsShot = 0;



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

    }

    @Override
    public void periodic() {
        if (NewPoseEstimatorSubsystem.getInstance().getCurrentPose().getTranslation().getDistance(new Translation2d(0,0)) < 4) {
            this.spin();
        }
        this.CountBalls();
    }

    public void spin() {
        motor2.setVoltage(3.0);
    }

    public void stopSpin() {
        motor2.setVoltage(0.0);
    }

    public void openPiston() {
        solenoid3.set(true);
    }
    
    public void closePiston() {
        solenoid3.set(false);
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

