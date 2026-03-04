package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public class RollersIOInputs {
    }

    public default void updateInputs(RollersIOInputs inputs) {}
    public default void setVoltage(double volts) {}
}
