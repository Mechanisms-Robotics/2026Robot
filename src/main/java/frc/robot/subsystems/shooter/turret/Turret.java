package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    public Turret(TurretIO io) {
        this.io = io;
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Turret", this.inputs);
    }

    /** @param angle robot relative turret angle */
    public void setAngle(Rotation2d angle) {
        this.io.setAngle(angle);
    }

    public void zero() {
        io.zero();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.inputs.positionDegrees);
    }
}
