package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public boolean isDeployed = false;
        public double dutyCycle = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setDeployed(boolean deployed) {}
    public default void setSpeed(double dutyCycle) {}

    public default void stop() {}
}
