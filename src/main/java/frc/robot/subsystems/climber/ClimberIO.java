package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean climberConnected = false;

        public double climberOutputPercent = 0.0;
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;

        public double climberVelocityRotationsPerSec = 0.0;

        public boolean jamDetected = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setMotorOpenLoop(double output) {}

    public default void setMotorVelocity(double speedRotPerSec) {}

    public default void stopMotor() {}
}