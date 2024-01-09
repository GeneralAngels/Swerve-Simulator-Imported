// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.NewDrive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock =
            new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private final List<Queue<Double>> queues = new ArrayList<>();
    private boolean isCANFD = false;

    public static final double ODOMETRY_FREQUENCY = 250.0;

    private static PhoenixOdometryThread instance = null;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
        start();
    }

    public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        signalsLock.lock();
        NewSwerveDriveSubsystem.odometryLock.lock();
        try {
            isCANFD = true;
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
            queues.add(queue);
        } finally {
            signalsLock.unlock();
            NewSwerveDriveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (isCANFD) {
                    BaseStatusSignal.waitForAll(2.0 / ODOMETRY_FREQUENCY, signals);
                } else {
                    // "waitForAll" does not support blocking on multiple
                    // signals with a bus that is not CAN FD, regardless
                    // of Pro licensing. No reasoning for this behavior
                    // is provided by the documentation.
                    Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY));
                    if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            NewSwerveDriveSubsystem.odometryLock.lock();
            try {
                for (int i = 0; i < signals.length; i++) {
                    queues.get(i).offer(signals[i].getValueAsDouble());
                }
            } finally {
                NewSwerveDriveSubsystem.odometryLock.unlock();
            }
        }
    }
}
