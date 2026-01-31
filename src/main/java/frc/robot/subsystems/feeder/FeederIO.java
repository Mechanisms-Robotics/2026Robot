package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Feeder subsystem. Keeps hardware-specific code isolated. */
public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        // Connection
        public boolean feederConnected = false;

        // Motor outputs / commands
        public double feederOutputPercent = 0.0; // open-loop request
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;

        // Sensors
        public double feederVelocityRotationsPerSec = 0.0;

        // Simple health / status
        public boolean jamDetected = false;
    }

    /** Populate periodic inputs for logging and state estimation. */
    public default void updateInputs(FeederIOInputs inputs) {}

    /** Run the feeder motor at the specified open loop percent [-1..1]. */
    public default void setMotorOpenLoop(double output) {}

    /** Run the feeder motor to the specified velocity (rotations per second). */
    public default void setMotorVelocity(double velocityRotPerSec) {}

    /** Stop the feeder motor (open-loop zero). */
    public default void stopMotor() {}
}
