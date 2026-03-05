package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
    @AutoLog
    public class IntakeIOInputs {
        public double velocityRPS;
        public double positionRotations;
        public double appliedVolts;
        public double armLeftAmps;
        public double armRightAmps;
        public boolean armLeftConnected = false;
        public boolean armRightConnected = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public default void setVoltage(double volts) {}
}
