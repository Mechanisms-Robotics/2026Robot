package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double rps; // rotations per second
    }
    
    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setVelocity(double rps) {}
    default void stopPower() {}
}
