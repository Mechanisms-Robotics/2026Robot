package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Flywheel", this.inputs);
    }

    /** @param rpm desired rotations per minute */
    public void setVelocity(double rpm) {
        this.io.setVelocity(rpm);
    }

    /** @return current rpm */
    public double getRPM() {
        return this.inputs.rpm;
    }

    public double getDesiredRPM() {
        return this.io.getDesiredVelocity();
    }
}
