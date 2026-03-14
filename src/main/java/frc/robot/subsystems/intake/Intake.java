package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SlapIO slapIO;
    private final RollersIO rollersIO;
    private final IntakeIOInputsAutoLogged slapInputs = new IntakeIOInputsAutoLogged();
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    public Intake(SlapIO slapIO, RollersIO rollersIO) {
        this.slapIO = slapIO;
        this.rollersIO = rollersIO;
    }

    @Override
    public void periodic() {
        this.slapIO.updateInputs(this.slapInputs);
        Logger.processInputs("Intake/Slap", this.slapInputs);

        this.rollersIO.updateInputs(this.rollersInputs);
        Logger.processInputs("Intake/Rollers", this.rollersInputs);

        if (this.slapInputs.positionDegrees < 20.0) {
            this.rollersIO.setDutyCycle(IntakeConstants.ROLLERS_DUTY_CYCLE);
        } else {
            this.rollersIO.setDutyCycle(0.0);
        }
    }

    /** Deploy the intake arms ans spin the rollers */
    public void deploy() {
        this.slapIO.setAngle(IntakeConstants.DEPLOY_ANGLE);
    }

    /** Stow the intake arms */
    public void stow() {
        this.slapIO.setAngle(IntakeConstants.STOW_ANGLE);
    }
}