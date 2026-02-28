package frc.robot.subsystems.shooter.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final Supplier<Rotation2d> robotHeadingSupplier;
    
    public Turret(TurretIO io, Supplier<Rotation2d> robotHeadingSupplier) {
        this.io = io;
        this.robotHeadingSupplier = robotHeadingSupplier;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public void setAngle(Rotation2d angle) {
        this.io.setAngle(angle.minus(robotHeadingSupplier.get()));
    }

    public void zero() {
        io.zero();
    }
}
