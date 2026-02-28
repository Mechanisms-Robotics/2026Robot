package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        double positionDegrees;
    }

    public default void updateInputs(HoodIOInputs inputs) {}
    public default void setAngle(Rotation2d angle) {}
}
