package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SlapIO {
    @AutoLog
    public class IntakeIOInputs {
        public double velocityDegreesPerSecond;
        public double positionDegrees;
        public double setPointDegrees;
        public double currentAmps = 0.0;
        public boolean leftConnected = false;
        public boolean rightConnected = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    /**
     * Set the angle the intake arms move toward
     * @param angle angle as a Rotation2d to start moving toward
     */
    public default void setAngle(Rotation2d angle) {}
}
