// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.CONSTANTS.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {

    private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static boolean isCANFD = new CANBus(
        DriveConstants.DRIVETRAIN_CONSTANTS.CANBusName
    ).isNetworkFD();
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
    }

    @Override
    public void start() {
        if (this.timestampQueues.size() > 0) {
            super.start();
        }
    }

    /** Registers a Phoenix signal to be read from the thread. */
    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        this.signalsLock.lock();
        Drivetrain.odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals =
                new BaseStatusSignal[this.phoenixSignals.length + 1];
            System.arraycopy(
                this.phoenixSignals,
                0,
                newSignals,
                0,
                this.phoenixSignals.length
            );
            newSignals[this.phoenixSignals.length] = signal;
            this.phoenixSignals = newSignals;
            this.phoenixQueues.add(queue);
        } finally {
            this.signalsLock.unlock();
            Drivetrain.odometryLock.unlock();
        }
        return queue;
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        this.signalsLock.lock();
        Drivetrain.odometryLock.lock();
        try {
            this.genericSignals.add(signal);
            this.genericQueues.add(queue);
        } finally {
            this.signalsLock.unlock();
            Drivetrain.odometryLock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drivetrain.odometryLock.lock();
        try {
            this.timestampQueues.add(queue);
        } finally {
            Drivetrain.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            // Wait for updates from all signals
            this.signalsLock.lock();
            try {
                if (isCANFD && this.phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(
                        2.0 / DriveConstants.ODOMETRY_FREQUENCY,
                        this.phoenixSignals
                    );
                } else {
                    // "waitForAll" does not support blocking on multiple signals with a bus
                    // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                    // behavior is provided by the documentation.
                    Thread.sleep(
                        (long) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY)
                    );
                    if (this.phoenixSignals.length > 0) {
                        BaseStatusSignal.refreshAll(this.phoenixSignals);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                this.signalsLock.unlock();
            }

            // Save new data to queues
            Drivetrain.odometryLock.lock();
            try {
                // Sample timestamp is current FPGA time minus average CAN latency
                //     Default timestamps from Phoenix are NOT compatible with
                //     FPGA timestamps, this solution is imperfect but close
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : this.phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (this.phoenixSignals.length > 0) {
                    timestamp -= totalLatency / this.phoenixSignals.length;
                }

                // Add new samples to queues
                for (int i = 0; i < this.phoenixSignals.length; i++) {
                    this.phoenixQueues.get(i).offer(
                        this.phoenixSignals[i].getValueAsDouble()
                    );
                }
                for (int i = 0; i < this.genericSignals.size(); i++) {
                    this.genericQueues.get(i).offer(
                        this.genericSignals.get(i).getAsDouble()
                    );
                }
                for (int i = 0; i < this.timestampQueues.size(); i++) {
                    this.timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                Drivetrain.odometryLock.unlock();
            }
        }
    }
}
