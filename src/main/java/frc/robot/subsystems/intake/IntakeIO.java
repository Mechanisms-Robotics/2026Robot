package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double armLeftDegrees;
        public double armRightDegrees;
        public double desiredDegrees;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void deploy() {}
    public default void retract() {}
    public default void setSpeed(double dutyCycle) {}
    public default void setAngle(Rotation2d angle) {}
    public default Rotation2d getAngle() {
        return Rotation2d.kZero;
    };

    public default void stop() {}
}
