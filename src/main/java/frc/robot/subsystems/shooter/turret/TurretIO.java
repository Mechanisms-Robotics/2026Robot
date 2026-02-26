package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public class TurretIOInputs {
        public double positionRadians = 0.0;
        public double velocityRadiansPerSec = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setPosition(Rotation2d position) {}

    public default void zero() {}
}
