package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Feeder subsystem. Keeps hardware-specific code isolated. */
public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        // Connection
        public boolean feederConnected = false;

        // Motor outputs / commands (three motors)
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;


        // Simple health / status
        public boolean jamDetected = false;
    }

    /** Populate periodic inputs for logging and state estimation. */
    public default void updateInputs(FeederIOInputs inputs) {}


    /** Run the feeder motor(s) at the specified open-loop voltage (volts).
     *  Default implementation may treat this as volts applied equally to all motors.
     *  Use positive/negative up to bus limits (typically +/-12V).
     */
    public default void setMotorOpenLoop(double volts) {}

    /** Stop the feeder motor (open-loop zero). */
    public default void stopMotor() {}
}
