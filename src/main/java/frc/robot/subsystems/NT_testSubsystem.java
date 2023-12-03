package frc.robot.subsystems;


import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NT_testSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    DoublePublisher output = NetworkTableInstance.getDefault().getTable("TEST").getDoubleTopic("output").publish();
    DoubleSubscriber input = NetworkTableInstance.getDefault().getTable("TEST").getDoubleTopic("input").subscribe(0);
    DoublePublisher _input_publisher = NetworkTableInstance.getDefault().getTable("TEST").getDoubleTopic("input").publish();

    /**
     * The Singleton instance of this NT_testSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static NT_testSubsystem INSTANCE = new NT_testSubsystem();

    /**
     * Returns the Singleton instance of this NT_testSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code NT_testSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static NT_testSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this NT_testSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private NT_testSubsystem() {

    }

    @Override
    public void periodic() {
        output.set(input.get());
    }
}

