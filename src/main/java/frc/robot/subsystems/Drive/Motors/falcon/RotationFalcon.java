// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive.Motors.falcon;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RotationFalcon extends Falcon {
    double homeAngle;
    double tick_0;
    double direction = 1;
    double offsetConstant;

    public Boolean isFlipeed = false;
    double drivingToDrivenGearRatio;

    int CANCoderPort;
    public CANCoder canCoder;

    public RotationFalcon(TalonFX talon, int CANCoderPort, int kPIDLoopIdx, double peakOutputForward, double peakOutputReverse, double Kf, double Kp, double Ki, double Kd, double homeAngle, double drivingToDrivenGearRatio, Boolean inverted, Boolean invertSensorPhase) {
        super(talon, kPIDLoopIdx, peakOutputForward, peakOutputReverse, Kf, Kp, Ki, Kd, 1);

        this.drivingToDrivenGearRatio = drivingToDrivenGearRatio;
        this.homeAngle = homeAngle;
        this.CANCoderPort = CANCoderPort;
        this.canCoder = new CANCoder(CANCoderPort);
    }

    public void changeFlipped() {
        isFlipeed = !isFlipeed;
    }

    @Override
    public void config(TalonFX _talon, int kPIDLoopSlotIdx, double peakOutputForward, double peakOutputReverse, double Kf, double Kp, double Ki, double Kd, Boolean invertSensorPhase) {
        // this._talon.configRemoteFeedbackFilter(this.CANCoderPort, RemoteSensorSource.CANCoder, 0);
        // this._talon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        _talon.configFactoryDefault();

        this._talon.setSensorPhase(invertSensorPhase);
        this._talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        this._talon.config_kF(kPIDLoopSlotIdx, Kf, kTimeoutMs);
        this._talon.config_kP(kPIDLoopSlotIdx, Kp, kTimeoutMs);
        this._talon.config_kI(kPIDLoopSlotIdx, Ki, kTimeoutMs);
        this._talon.config_kD(kPIDLoopSlotIdx, Kd, kTimeoutMs);
        // _talon.configAllowableClosedloopError(0, kPIDLoopSlotIdx, kTimeoutMs);
        this._talon.selectProfileSlot(kPIDLoopSlotIdx, 0); // pidIdx = 0 because we use regular closed loop controller
        // this._talon.configFeedbackNotContinuous(true, kTimeoutMs);
    }

    public void setAngle(double angle) {
        this.setPosition(oldOptimize(angle));
    }

    public double oldOptimize(double angle) {
        double wantedTick = this.anglesToTicks(angle) + tick_0;
        double curretPosition = this.getPosition();

        double error = wantedTick - curretPosition;
        double finalTarget;

        if (error > this.anglesToTicks(180)) {
            finalTarget = wantedTick - this.anglesToTicks(360);
        } else if (error < this.anglesToTicks(-180)) {
            finalTarget = wantedTick + this.anglesToTicks(360);
        } else {
            finalTarget = wantedTick;
        }
        return finalTarget;
    }

    public double newOptimize(double angle) {
        double curretPosition = this.getPosition();
        double currentAngle = this.getAngleByFalcon();
        double wantedTick;

        double error1 = (angle - currentAngle);
        double error2 = -(360 * Math.signum(error1) - error1);

        if (Math.abs(error1) < Math.abs(error2)) {
            wantedTick = curretPosition + anglesToTicks(error1);
        } else {
            wantedTick = curretPosition + anglesToTicks(error2);
        }

        return wantedTick;
    }

    public void setEncoder() {
        double a = this.anglesToTicks(this.homeAngle - this.canCoder.getAbsolutePosition());
        double y = this._talon.getSelectedSensorPosition(0);

        tick_0 = y - a;
    }

    public double anglesToTicks(double angles) {
        return angles * 2048 / 360 * drivingToDrivenGearRatio;
    }

    public double ticksToAngle(double ticks) {
        return ((ticks / 2048) * 360 / drivingToDrivenGearRatio);
    }

    public double getAngleByCanCoder() {
        // by canCoder
        return this.canCoder.getAbsolutePosition() - this.homeAngle;
    }

    public double getAngleByFalcon() {
        return ticksToAngle(this._talon.getSelectedSensorPosition() - tick_0);
    }

    @Override
    public void setRpm(double rpm) {
        super.setRpm(rpm * direction);
    }
}