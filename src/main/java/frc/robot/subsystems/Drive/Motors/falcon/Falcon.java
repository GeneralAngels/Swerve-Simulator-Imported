// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive.Motors.falcon;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.time.StopWatch;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Robot;


/**
 * Add your docs here.
 */
public class Falcon {
    public TalonFX _talon;
    public TalonFXSimCollection sim_falcon;

    // Contstans:
    int kTimeoutMs = 30; // TODO: validate that this is the right way
    int ticksForRotation = 2048;
    public double rpmToUnitsRatioVelocity = ticksForRotation / 600; // 600 = 100ms / minute

    public double current_encoder = 0;

    public Falcon(TalonFX talon, int kPIDLoopSlotIdx, double peakOutputForward, double peakOutputReverse, double Kf, double Kp, double Ki, double Kd, double MotorToActualRatio) {
        this._talon = talon;
        sim_falcon = _talon.getSimCollection();

        this.rpmToUnitsRatioVelocity = this.rpmToUnitsRatioVelocity * MotorToActualRatio;

        this._talon.configNeutralDeadband(0.001); // setting the minimal Deadband

        this.config(_talon, kPIDLoopSlotIdx, peakOutputForward, peakOutputReverse, Kf, Kp, Ki, Kd, false);
    }

    public void config(TalonFX _talon, int kPIDLoopSlotIdx, double peakOutputForward, double peakOutputReverse, double Kf, double Kp, double Ki, double Kd, Boolean invertSensorPhase) {
        // Setting feedback sensor (encoder)
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopSlotIdx, kTimeoutMs);

        // Config peak and minimal values:
        _talon.configNominalOutputForward(0, kTimeoutMs);
        _talon.configNominalOutputReverse(0, kTimeoutMs);
        _talon.configPeakOutputForward(peakOutputForward, kTimeoutMs);
        _talon.configPeakOutputReverse(peakOutputReverse, kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        _talon.config_kF(kPIDLoopSlotIdx, Kf, kTimeoutMs);
        _talon.config_kP(kPIDLoopSlotIdx, Kp, kTimeoutMs);
        _talon.config_kI(kPIDLoopSlotIdx, Ki, kTimeoutMs);
        _talon.config_kD(kPIDLoopSlotIdx, Kd, kTimeoutMs);
        _talon.selectProfileSlot(kPIDLoopSlotIdx, 0); // pidIdx = 0 because we use regular closed loop controller
    }

    public void setRpm(double Rpm) {
        if (Robot.isSimulation()) {
            sim_falcon.setIntegratedSensorVelocity((int) (Rpm * rpmToUnitsRatioVelocity));
        }
        this._talon.set(TalonFXControlMode.Velocity, Rpm * rpmToUnitsRatioVelocity);
    }

    public void setPrecentage(double precentage) {
        this._talon.set(TalonFXControlMode.PercentOutput, precentage);
    }

    public void setPosition(double position) {
        if (Robot.isSimulation()) {
            sim_falcon.setIntegratedSensorRawPosition((int) position);
        }
        this._talon.set(TalonFXControlMode.Position, position);
    }

    public double getVelocityTicks() {
        return this._talon.getSelectedSensorVelocity(0);
    }

    public double getRpm() {
        return this.getVelocityTicks() / rpmToUnitsRatioVelocity;
    }

    public double getPosition() {
        return this._talon.getSelectedSensorPosition();
    }

    public double getKf(double precent) {
        this.setPrecentage(precent);

        StopWatch stopWatch = new StopWatch();
        stopWatch.start();

        while (stopWatch.getDuration() < 4) {
        }

        this.setPrecentage(0);
        return (precent * 1023) / this.getVelocityTicks();
    }

}
