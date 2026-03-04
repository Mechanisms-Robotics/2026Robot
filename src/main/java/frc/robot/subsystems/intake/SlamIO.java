package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
    @AutoLog
    public class IntakeIOInputs {
        public double velocityRPS;
        public double positionRotations;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setVoltage(double volts) {}

    public default void stop() {}
}
