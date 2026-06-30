package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double positionDegrees;
        public double setpointDegrees;
        public double appliedVolts;
        public double currentAmps;
        public double tempFahrenheit;
        public boolean connected = false;
    }

    public default void updateInputs(HoodIOInputs inputs) {}
    public default void setAngle(Rotation2d angle) {}
    public default void changeAngle(Rotation2d delta) {}
}
