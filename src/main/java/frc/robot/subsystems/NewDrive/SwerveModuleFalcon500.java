package frc.robot.subsystems.NewDrive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

public class SwerveModuleFalcon500 {
    public static final int DRIVE_CURRENT_LIMIT = 70;
    public static final int STEER_CURRENT_LIMIT = 30;

    // Constants specific to the hardware
    /**
     * Radius of the wheel. Can be used to figure out distance data
     */
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.95);

    /**
     * From motor rotations to the wheel revolutions
     */
    private static final double DRIVE_GEAR_RATIO_JACK_IN_THE_BOT = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double DRIVE_GEAR_RATIO = 6.75;

    /**
     * Conversion constant: From motor encoder ticks to position data (m)
     * <p>
     * motor encoder ticks -> meters: motor encoder ticks * constant
     * meters -> motor encoder ticks: meters / constant
     */
    private static final double DRIVE_SENSOR_POSITION_COEFFICIENT = (2 * Math.PI * WHEEL_RADIUS_METERS)
            / (2048 * DRIVE_GEAR_RATIO);

    Rotation2d relative_offset = Rotation2d.fromDegrees(0);

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
    TalonFXSimState simDriveMotor;
    /**
     * TalonFX swerve module steer motor
     */
    TalonFX steerMotor;
    TalonFXSimState simSteerMotor;
    /**
     * Swerve module steer encoder (absolute angular position)
     */
    CANCoder steerEncoder;

    VelocityVoltage velocity_request = new VelocityVoltage(0).withSlot(0).withAcceleration(0).withEnableFOC(false);
    PositionVoltage position_request = new PositionVoltage(0).withSlot(0).withVelocity(0);

    public static class ModuleInputs {
        public double[] odometryDrivePositionsMeters = new double[]{};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
    }

    ModuleInputs inputs = new ModuleInputs();

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;

    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;

    private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[]{};
    private double lastPositionMeters = 0.0; // Used for delta calculation

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
        driveMotor = new TalonFX(driveMotorId, "canivore");
        simDriveMotor = driveMotor.getSimState();

        steerMotor = new TalonFX(steerMotorId, "canivore");
        simSteerMotor = steerMotor.getSimState();

        steerEncoder = new CANCoder(steerCanCoderID, "canivore");

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        driveConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        driveConfig.Slot0.kI = 0.001; // An error of 1 rotation per second increases output by 0.5V every second
        driveConfig.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        driveConfig.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        driveConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /*
        driveConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        driveConfig.slot0.kP = SwerveConstants.Drive_Kp;
        driveConfig.slot0.kF = SwerveConstants.Drive_Kf;
        driveConfig.supplyCurrLimit.currentLimit = DRIVE_CURRENT_LIMIT;
        driveConfig.supplyCurrLimit.enable = true;

        driveMotor.setInverted(TalonFXInvertType.Clockwise);
        driveMotor.setNeutralMode(NeutralMode.Brake);
         */

        driveMotor.getConfigurator().apply(driveConfig);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.Slot0.kP = 1.3; // An error of 1 rotation per second results in 2V output
        steerConfig.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
        steerConfig.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        steerConfig.Slot0.kV = 0.0; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        steerConfig.CurrentLimits.SupplyCurrentLimit = STEER_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.Feedback.SensorToMechanismRatio = -1; // Instead of: steerMotor.setSensorPhase(true);
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steerMotor.getConfigurator().apply(steerConfig);

        CANCoderConfiguration steerEncoderConfig = new CANCoderConfiguration();
        steerEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        steerEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        steerEncoderConfig.magnetOffsetDegrees = Units.radiansToDegrees(steerAngleOffsetRad);
        // steerEncoderConfig.magnetOffsetDegrees = 0;

        steerEncoder.configAllSettings(steerEncoderConfig);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();

        /* Fast odometry init */
        drivePosition = driveMotor.getPosition();
        drivePositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());

        turnPosition = steerMotor.getPosition();
        turnPositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(steerMotor, steerMotor.getPosition());

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.PoseEstimatorConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);

        steerMotor.setPosition(0);

//        if (Utils.isSimulation()) {
//            driveMotor.getRotorVelocity().setUpdateFrequency(1000);
//            driveMotor.getRotorPosition().setUpdateFrequency(1000);
//            steerMotor.getRotorPosition().setUpdateFrequency(1000);
//        }
    }

    public void updateOdometryInputs() {
        // In radians with consideration of gear.
        inputs.odometryDrivePositionsMeters =
                drivePositionQueue.stream()
                        .mapToDouble((Double value) ->
                                value * (2 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO)
                        .toArray();

        // In Rotation2d.
        inputs.odometryTurnPositions =
                turnPositionQueue.stream()
                        .map((Double value) -> Rotation2d.fromRotations(value / STEER_GEAR_RATIO).plus(relative_offset))
                        .toArray(Rotation2d[]::new);

        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    public double[] getDrivePositionArray() {
        return inputs.odometryDrivePositionsMeters;
    }

    public Rotation2d[] getTurnPositions() {
        return inputs.odometryTurnPositions;
    }

    public void updateStatus(String name) {
    }

    public void setTargetSteerPosition(double targetSteerPositionRad) {
//         steerMotor.set(TalonFXControlMode.Position, targetSteerPositionRad / STEER_SENSOR_POSITION_COEFFICIENT);

        targetSteerPositionRad = targetSteerPositionRad - relative_offset.getRadians();

        steerMotor.setControl(position_request.withPosition(
                targetSteerPositionRad * STEER_GEAR_RATIO / (2 * Math.PI)
        ));

        if (Robot.isSimulation()) {
            simSteerMotor.setRawRotorPosition(
                    targetSteerPositionRad * STEER_GEAR_RATIO / (2 * Math.PI));
        }
    }

    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
//        driveMotor.set(TalonFXControlMode.Velocity, targetDriveVelocityMetersPerSec / DRIVE_SENSOR_VELOCITY_COEFFICIENT);
        driveMotor.setControl(velocity_request.withVelocity(targetDriveVelocityMetersPerSec * DRIVE_GEAR_RATIO / (2 * Math.PI * WHEEL_RADIUS_METERS)));

        if (Robot.isSimulation()) {
            simDriveMotor.setRotorVelocity(
                    targetDriveVelocityMetersPerSec * DRIVE_GEAR_RATIO / (2 * Math.PI * WHEEL_RADIUS_METERS));
        }
    }

    public void setState(SwerveModuleState targetState) {
//        var steerPositionTicks = steerMotor.getSelectedSensorPosition();
//        var steerPositionRad = steerPositionTicks * STEER_SENSOR_POSITION_COEFFICIENT;

        var currentAngleRadians = steerMotor.getPosition().getValueAsDouble() / STEER_GEAR_RATIO * (2 * Math.PI);
        currentAngleRadians += relative_offset.getRadians();

        double angleError = getAngleError(targetState.angle.getRadians(), currentAngleRadians);

        double resultAngle = currentAngleRadians
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

            double currentPosition = steerMotor.getPosition().getValueAsDouble(); // in rotations.
            double currentAngle = Units.rotationsToRadians(currentPosition) / STEER_GEAR_RATIO;

            double absoluteEncoderAngle = steerEncoder.getAbsolutePosition();

            double angle_error = getAngleError(Units.degreesToRadians(absoluteEncoderAngle), currentAngle); // in radians.

            relative_offset = Rotation2d.fromRadians(angle_error);


            // steerMotor.setPosition(
            // currentPosition + Units.radiansToRotations(angle_error) * STEER_GEAR_RATIO);
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble() / DRIVE_GEAR_RATIO * (2 * Math.PI * WHEEL_RADIUS_METERS),
                Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble() / STEER_GEAR_RATIO).plus(relative_offset));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveMotor.getVelocity().getValueAsDouble() / DRIVE_GEAR_RATIO * (2 * Math.PI * WHEEL_RADIUS_METERS),
                Rotation2d.fromRadians(steerMotor.getPosition().getValueAsDouble() / STEER_GEAR_RATIO * (2 * Math.PI)).plus(relative_offset));
    }

    public void updateSim(double looperDt) {
        var current_velocity = driveMotor.getRotorVelocity().getValueAsDouble(); // in rotation per second.

        simDriveMotor.addRotorPosition(
                current_velocity * looperDt
        );
    }

    public void showSimOutputs(int index) {
        var current_state = getState();
        var current_position = getPosition();

        SmartDashboard.putNumber(index + ".velocity", current_state.speedMetersPerSecond);
        SmartDashboard.putNumber(index + ".angle", current_state.angle.getDegrees());
        SmartDashboard.putNumber(index + ".encoder_meters", current_position.distanceMeters);
    }
}
