package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public class TurretIOInputs {
        public double positionRadians = 0.0;
        public double velocityRadiansPerSec = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setPosition(double positionRadians) {}
}
