package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.NewDrive.NewSwerveDriveSubsystem;


public class DefaultDriveCommand extends Command {
    private final NewSwerveDriveSubsystem swerve = NewSwerveDriveSubsystem.getInstance();
    CommandPS4Controller controller;

    double maxOmega = 3.95;
    double maxSpeed = 4.15;

    ChassisSpeeds swerveSpeeds = new ChassisSpeeds();

    public DefaultDriveCommand(CommandPS4Controller driver_controller) {
        addRequirements(swerve);
        this.controller = driver_controller;
    }

    public double getX() {
        double value = -this.controller.getLeftY();
        if (Math.abs(value) < 0.05) {
            value = 0;
        }
        return value;
    }

    public double getY() {
        double value = -this.controller.getLeftX();
        if (Math.abs(value) < 0.05) {
            value = 0;
        }
        return value;
    }

    public double getOmega() {
        double value = -this.controller.getRightX();
        if (Math.abs(value) < 0.05) {
            value = 0;
        }
        return value * maxOmega;
    }

    public double getGas() {
        var vector_speed = Math.hypot(getX(), getY());

        if (vector_speed < 0.05) {
            return 0;
        }

        if (this.controller.getR2Axis() < -0.9) {
            return vector_speed * 0.2; // speed without gas
        }

        return (this.controller.getR2Axis() + 1) / 2 * maxSpeed;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        var angle = Math.atan2(getY(), getX());
        var speed = getGas();

        swerveSpeeds.vxMetersPerSecond = Math.cos(angle) * speed;
        swerveSpeeds.vyMetersPerSecond = Math.sin(angle) * speed;
        swerveSpeeds.omegaRadiansPerSecond = getOmega();

        NewSwerveDriveSubsystem.getInstance().setAbsoluteVelocities(swerveSpeeds);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        swerveSpeeds.vxMetersPerSecond = 0;
        swerveSpeeds.vyMetersPerSecond = 0;
        swerveSpeeds.omegaRadiansPerSecond = 0;

        NewSwerveDriveSubsystem.getInstance().setRelativeVelocities(swerveSpeeds);
    }
}
