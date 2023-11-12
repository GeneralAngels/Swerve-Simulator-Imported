package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.SwerveDriveTrain;


public class DriveSpeedsWithOmega extends Command {
    SwerveDriveTrain swerveDriveTrain;
    ChassisSpeeds chassisSpeeds;

    double setpoint;

    /** Creates a new DriveSpeeds. */
    public DriveSpeedsWithOmega(SwerveDriveTrain swerveDriveTrain, ChassisSpeeds chassisSpeeds, double setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDriveTrain);

        this.swerveDriveTrain = swerveDriveTrain;
        this.chassisSpeeds = chassisSpeeds;
        this.setpoint = setpoint;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        chassisSpeeds.omegaRadiansPerSecond = swerveDriveTrain.getOmegaCorrection(setpoint, 3);
        this.swerveDriveTrain.setWpiAbsoluteVelocoties(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDriveTrain.setWpiAbsoluteVelocoties(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
