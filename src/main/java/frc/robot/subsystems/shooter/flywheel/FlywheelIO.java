package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double rpm;
        public double desiredRpm;
        public double leaderTempFahrenheit;
        public double followerTempFahrenheit;
        public double leaderCurrentAmps;
        public double followerCurrentAmps;
        public boolean leaderConnected = false;
        public boolean followerConnected = false;
    }
    
    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setVelocity(double rpm) {}

    default void setVoltage(double voltage) {}

    default double getDesiredVelocity() {
        return 0.0;
    }
}
