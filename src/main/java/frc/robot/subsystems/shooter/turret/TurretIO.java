package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public class TurretIOInputs {
        public double positionDegrees = 0.0;
        public double velocityDegreesPerSecond = 0.0;
        public double setpointDegrees = 0.0;
        public double current = 0.0;
        public boolean connected = false;
        public double tempFahrenheit = 0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setAngle(Rotation2d position) {}

    public default void zero() {}
}
