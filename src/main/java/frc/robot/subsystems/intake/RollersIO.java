package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public class RollersIOInputs {
        public double currentAmps = 0.0;
        public boolean isConnected = false;
        public double tempFahrenheit = 0;
    }

    public default void updateInputs(RollersIOInputs inputs) {}
    public default void setDutyCycle(double dutyCycle) {}
}
