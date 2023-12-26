package frc.robot.subsystems.NewDrive;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.time.StopWatch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.SwerveConstants;

public class SwerveModuleFalcon500 {
    public static final int DRIVE_CURRENT_LIMIT = 70;
    public static final int STEER_CURRENT_LIMIT = 30;

    // Constants specific to the hardware
    /**
     * Radius of the wheel. Can be used to figure out distance data
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.95);

    /**
     * From motor rotations to the wheel revolutions
     */
    private static final double DRIVE_GEAR_RATIO_JACK_IN_THE_BOT = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double DRIVE_GEAR_RATIO = 6.75;

    /**
     * Conversion constant: From motor encoder ticks to position data (m)
     * <p>
     * motor encoder ticks -> meters: motor encoder ticks * constant
     * meters -> motor encoder ticks: meters / constant
     */
    private static final double DRIVE_SENSOR_POSITION_COEFFICIENT = (2 * Math.PI * WHEEL_RADIUS_METERS)
            / (2048 * DRIVE_GEAR_RATIO);

    /**
     * Conversion constant: From motor encoder ticks to velocity data (m/s)
     * <p>
     * motor encoder ticks / 100 ms -> meters per second: motor encoder ticks *
     * constant
     * meters per second -> motor encoder ticks / 100 ms: meters per second /
     * constant
     */
    private static final double DRIVE_SENSOR_VELOCITY_COEFFICIENT = DRIVE_SENSOR_POSITION_COEFFICIENT * 10;

    /**
     * From motor rotations to the wheel rotations (150 / 7 motor rotations : 1 full
     * rotation of the wheel [2π])
     */
    private static final double STEER_GEAR_RATIO = 150.0 / 7;

    /**
     * Conversion constant: From motor encoder ticks to angle data (steer position)
     * (radians)
     * <p>
     * motor encoder ticks -> radians: motor encoder ticks * constant
     * radians -> motor encoder ticks: radians / constant
     */
    private static final double STEER_SENSOR_POSITION_COEFFICIENT = (2 * Math.PI) / (2048 * STEER_GEAR_RATIO);

    /**
     * Conversion constant: From motor encoder ticks to angular velocity data (steer
     * velocity) (radians/s)
     * <p>
     * motor encoder ticks / 100 ms -> radians per second: motor encoder ticks / 100
     * ms * constant
     * radians per second -> motor encoder ticks / 100 ms: radians per second /
     * constant
     */
    private static final double STEER_SENSOR_VELOCITY_COEFFICIENT = STEER_SENSOR_POSITION_COEFFICIENT * 10;

    private static final double VELOCITY_COEFFICIENT = 2;

    // Hardware object initialization
    /**
     * TalonFX swerve module drive motor
     */
    TalonFX driveMotor;
    TalonFXSimCollection simDriveMotor;
    /**
     * TalonFX swerve module steer motor
     */
    TalonFX steerMotor;
    TalonFXSimCollection simSteerMotor;
    /**
     * Swerve module steer encoder (absolute angular position)
     */
    CANCoder steerEncoder;

    String name = "";

    /**
     * Initializes the motors, encoder, and the settings for each of the devices.
     *
     * @param driveMotorId        Drive motor CAN ID
     * @param steerMotorId        Steer motor CAN ID
     * @param steerCanCoderID     Steer encoder CAN ID
     * @param steerAngleOffsetRad This is the offset applied to the angle motor's
     *                            absolute encoder so that a reading of 0 degrees
     *                            means the module is facing forwards.
     */
    public SwerveModuleFalcon500(
            int driveMotorId, int steerMotorId, int steerCanCoderID, double steerAngleOffsetRad) {
        driveMotor = new TalonFX(driveMotorId);
        simDriveMotor = driveMotor.getSimCollection();

        steerMotor = new TalonFX(steerMotorId);
        simSteerMotor = steerMotor.getSimCollection();

        steerEncoder = new CANCoder(steerCanCoderID);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        driveConfig.slot0.kP = SwerveConstants.Drive_Kp;
        driveConfig.slot0.kF = SwerveConstants.Drive_Kf;
        driveConfig.supplyCurrLimit.currentLimit = DRIVE_CURRENT_LIMIT;
        driveConfig.supplyCurrLimit.enable = true;

        driveMotor.configAllSettings(driveConfig);
        driveMotor.setInverted(TalonFXInvertType.Clockwise);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        steerConfig.slot0.kP = SwerveConstants.Rotation_Kp; // Jack in the bot: 0.2
        steerConfig.supplyCurrLimit.currentLimit = STEER_CURRENT_LIMIT;
        steerConfig.supplyCurrLimit.enable = true;

        steerMotor.configAllSettings(steerConfig);
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(TalonFXInvertType.Clockwise);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        CANCoderConfiguration steerEncoderConfig = new CANCoderConfiguration();
        steerEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        steerEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        steerEncoderConfig.magnetOffsetDegrees = Units.radiansToDegrees(steerAngleOffsetRad);
        // steerEncoderConfig.magnetOffsetDegrees = 0;

        steerEncoder.configAllSettings(steerEncoderConfig);
    }

    public void updateStatus(String name) {

    }

    public void setTargetSteerPosition(double targetSteerPositionRad) {
        steerMotor.set(TalonFXControlMode.Position, targetSteerPositionRad / STEER_SENSOR_POSITION_COEFFICIENT);

        if (Robot.isSimulation()) {
            simSteerMotor.setIntegratedSensorRawPosition(
                    -(int) (targetSteerPositionRad / STEER_SENSOR_POSITION_COEFFICIENT));
        }
    }

    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
        driveMotor.set(
                TalonFXControlMode.Velocity, targetDriveVelocityMetersPerSec / DRIVE_SENSOR_VELOCITY_COEFFICIENT);

        if (Robot.isSimulation()) {
            simDriveMotor.setIntegratedSensorVelocity(
                    -(int) (targetDriveVelocityMetersPerSec / DRIVE_SENSOR_VELOCITY_COEFFICIENT));
        }
    }

    public void setState(SwerveModuleState targetState) {
        var steerPositionTicks = steerMotor.getSelectedSensorPosition();
        var steerPositionRad = steerPositionTicks * STEER_SENSOR_POSITION_COEFFICIENT;

        double currentAngle = steerPositionRad; // Current angle of the swerve module
        double angleError = getAngleError(targetState.angle.getRadians(), currentAngle);

        double resultAngle = currentAngle
                + angleError; // Adding that distance to our current angle (directly from the steer encoder).
                              // Becomes
        // our target angle

        setTargetDriveVelocity(targetState.speedMetersPerSecond);
        setTargetSteerPosition(resultAngle);
    }

    static double getAngleError(double _targetAngleRadians, double currentAngle) {
        double targetAngle = MathUtil.inputModulus(
                _targetAngleRadians,
                0,
                2 * Math.PI); // Target angle of the swerve module, limited to a domain between 0 and 2π.

        double absoluteAngle = MathUtil.inputModulus(
                currentAngle, 0, 2 * Math.PI); // Limiting the domain of the current angle to a domain of 0 to 2π.

        double angleError = MathUtil.inputModulus(
                targetAngle - absoluteAngle,
                -Math.PI,
                Math.PI); // Finding the difference in between the current and target angle (in radians).

        return angleError;
    }

    public void resetToAbsolute() {
        if (Robot.isReal()) {
            double currentPosition = steerMotor.getSelectedSensorPosition();
            double absoluteEncoderAngle = steerEncoder.getAbsolutePosition();

            double angle_error = getAngleError(Units.degreesToRadians(absoluteEncoderAngle), currentPosition * STEER_SENSOR_POSITION_COEFFICIENT);

            steerMotor.setSelectedSensorPosition(
                    currentPosition + angle_error / STEER_SENSOR_POSITION_COEFFICIENT);
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getSelectedSensorPosition() * DRIVE_SENSOR_POSITION_COEFFICIENT,
                Rotation2d.fromRadians(steerMotor.getSelectedSensorPosition() * STEER_SENSOR_POSITION_COEFFICIENT));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveMotor.getSelectedSensorVelocity() * DRIVE_SENSOR_VELOCITY_COEFFICIENT,
                Rotation2d.fromRadians(steerMotor.getSelectedSensorPosition() * STEER_SENSOR_POSITION_COEFFICIENT));
    }

    public void updateSim(double looperDt) {
        var current_velocity = -driveMotor.getSelectedSensorVelocity(); // in ticks / 100ms
        var current_velocity_in_seconds = current_velocity * 10; // in ticks / second
        int current_encoder_change = (int) (current_velocity_in_seconds * looperDt);

        simDriveMotor.addIntegratedSensorPosition(current_encoder_change);
    }

    public void showSimOutputs(int index) {
        var current_state = getState();
        var current_position = getPosition();

        SmartDashboard.putNumber(index + ".velocity", current_state.speedMetersPerSecond);
        SmartDashboard.putNumber(index + ".angle", current_state.angle.getDegrees());
        SmartDashboard.putNumber(index + ".encoder_meters", current_position.distanceMeters);
    }
}
