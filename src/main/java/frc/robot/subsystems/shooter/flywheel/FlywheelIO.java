package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double rpm;
        public double desiredRpm;
    }
    
    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setVelocity(double rpm) {}

    default double getDesiredVelocity() {
        return 0.0;
    }
}
