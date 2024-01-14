// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.time.StopWatch;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Utils.LimelightMeasurement;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NT_testSubsystem;
import frc.robot.subsystems.NewDrive.SwerveModuleFalcon500;
import frc.robot.subsystems.NewDrive.NewPoseEstimatorSubsystem;
import frc.robot.subsystems.utils.NT_Helper;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;
import frc.robot.subsystems.NewDrive.Resetter;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    Subsystem swerve_resetter = new Resetter();

    StopWatch autonomous_stopwatch = new StopWatch();
    DoublePublisher stopwatch_nt = NetworkTableInstance.getDefault().getTable("autonomous").getDoubleTopic("stopwatch").publish();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        if (isReal()) {
            // Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables
        } else {
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

            /*
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
             */
        }

        // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        m_robotContainer = new RobotContainer();
        NT_testSubsystem.getInstance();
        NewPoseEstimatorSubsystem.getInstance().setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        compressor.enableDigital();

        SmartDashboard.putData("command-2-3", new InstantCommand(() -> {
            System.out.println("\n\n");
            System.out.println("command-----command");
            System.out.println("\n\n");
        }));
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated upating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

//        NewSwerveDriveSubsystem.getInstance().getAllCanCoders();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
//        m_robotContainer.newSwerve.setRelativeVelocities(new ChassisSpeeds());
        NewSwerveDriveSubsystem.getInstance().setAbsoluteVelocities(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomous_stopwatch.start();
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
        stopwatch_nt.set(autonomous_stopwatch.getDuration());
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
        LimelightMeasurement limelightMeasurement = Limelight.MegaTagEstimate();
        if (limelightMeasurement != null) {
            NewPoseEstimatorSubsystem.getInstance().setCurrentPose(limelightMeasurement.pose);
        }



        NewSwerveDriveSubsystem.getInstance().setDefaultCommand(
                new DefaultDriveCommand(
                        m_robotContainer.driver
                )
        );

        this.m_robotContainer.shooter_rig.renew();
        m_robotContainer.shooter_rig.slewRateLimiter.reset(0);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        m_robotContainer.shooter_rig.teleopPeriodicPercent();
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
        CommandScheduler.getInstance().run();
    }
}