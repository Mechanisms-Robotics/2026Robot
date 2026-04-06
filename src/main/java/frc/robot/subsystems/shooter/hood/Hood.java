package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.HoodConstants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Hood", this.inputs);
    }

    public void setAngle(Rotation2d angle) {
        Logger.recordOutput("Hood/desiredAngle", angle);
        this.io.setAngle(angle);
    }

    public void changeAngle(Rotation2d delta) {
        this.io.changeAngle(delta);
    }

    public void stow() {
        setAngle(Rotation2d.fromDegrees(HoodConstants.MIN_DEGREES));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.inputs.positionDegrees);
    }
}
