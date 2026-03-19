package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SlapIO slapIO;
    private final RollersIO rollersIO;
    private final SlapIOInputsAutoLogged slapInputs = new SlapIOInputsAutoLogged();
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

        double rollerAngleThreshold = 45.0;
        if (this.slapInputs.positionDegrees < rollerAngleThreshold && this.slapInputs.setpointDegrees < rollerAngleThreshold) {
            this.runRollers();
        } else {
            this.stopRollers();
        }
    }

    public void runRollers() {
        this.rollersIO.setDutyCycle(IntakeConstants.ROLLERS_DUTY_CYCLE);
    }

    public void stopRollers() {
        this.rollersIO.setDutyCycle(0.0);
    }

    /** Deploy the intake arms ans spin the rollers */
    public void deploy() {
        this.slapIO.setAngle(IntakeConstants.DEPLOY_ANGLE);
        //runRollers();

    }

    /** Stow the intake arms */
    public void stow() {
        this.slapIO.setAngle(IntakeConstants.STOW_ANGLE);
        //stopRollers();
    }
}