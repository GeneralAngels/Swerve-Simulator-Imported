// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils.LimelightMeasurement;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NT_testSubsystem;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    PneumaticHub ph = new PneumaticHub(1);
    Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    PneumaticsControlModule pcm = new PneumaticsControlModule();

    DigitalInput beam_break = new DigitalInput(7);

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", "General Angels 2230");

        // Set up data receivers & replay source
        if (Robot.isReal()) {
            System.out.println("INITIALIZING LOGS");
            // Running on a real robot, log to a USB stick ("/U/logs")
            // try { Logger.addDataReceiver(new WPILOGWriter("/U/logs"));}
            // catch (Exception e) {System.out.println("ERRORING!");; System.out.println(e); }
            
            Logger.addDataReceiver(new NT4Publisher());
        }

        else if (Robot.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        }


        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();

        m_robotContainer = new RobotContainer();
        // Shooter.getInstance();
        NT_testSubsystem.getInstance();
        NewPoseEstimatorSubsystem.getInstance().setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        //compressor.enableAnalog(5, 40);
        //ph.enableCompressorAnalog(5, 40);

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated upating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        Logger.recordOutput("pressure switch", pcm.getPressureSwitch());

        Logger.recordOutput("beam break", beam_break.get());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        // m_robotContainer.newSwerve.setRelativeVelocities(new ChassisSpeeds());
        NewSwerveDriveSubsystem.getInstance().setAbsoluteVelocities(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        LimelightMeasurement newPose = Limelight.MegaTagEstimate();
        if (newPose != null) {
            NewPoseEstimatorSubsystem.getInstance().setCurrentPose(newPose.pose);
        }

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        NewPoseEstimatorSubsystem.getInstance().setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        LimelightMeasurement limelightMeasurement = Limelight.MegaTagEstimate();
        if (limelightMeasurement != null) {
            NewPoseEstimatorSubsystem.getInstance().setCurrentPose(limelightMeasurement.pose);
        }

        NewSwerveDriveSubsystem.getInstance().setDefaultCommand(
                new DefaultDriveCommand(
                        m_robotContainer.driver));

        m_robotContainer.shooter_rig.slewRateLimiter.reset(0);


    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //m_robotContainer.shooter_rig.teleopPeriodicPercent();
        Logger.recordOutput("Analog Pressure Sensor", compressor.getPressure());


    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
