package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public void setPosition(double positionRadians) {
        Logger.recordOutput("Hood/desiredPositionRadians", positionRadians);
        this.io.setPosition(positionRadians);
    }
}
