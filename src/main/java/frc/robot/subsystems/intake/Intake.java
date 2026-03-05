package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.CONSTANTS.IntakeConstants.SlamState;

public class Intake extends SubsystemBase {
    private final SlamIO slamIO;
    private final RollersIO rollersIO;
    private final IntakeIOInputsAutoLogged slamInputs = new IntakeIOInputsAutoLogged();
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    SlamState state = SlamState.RETRACT_VOLTS;

    // the way our motors are configured the positive is the retracted direction
    private double retractedPositionDetected = Double.MIN_VALUE;

    public Intake(SlamIO io, RollersIO rollersIO) {
        this.slamIO = io;
        this.rollersIO = rollersIO;
    }

    @Override
    public void periodic() {
        this.slamIO.updateInputs(this.slamInputs);
        Logger.processInputs("Intake/Slam", this.slamInputs);

        this.rollersIO.updateInputs(this.rollersInputs);
        Logger.processInputs("Intake/Rollers", this.rollersInputs);

        // Capture the most retracted position. If we accidentally start
        // with the intake out it's okay. It probably won't retract until
        // we manually close it and capture the new retracted position.
        if (this.slamInputs.positionRotations > retractedPositionDetected) {
            retractedPositionDetected = this.slamInputs.positionRotations;
        }

        // use voltage control but dampen speed

        double feedForward = 0.0;

        if (state == SlamState.RETRACT_VOLTS) { // only use feedforward on retraction
            // the deployedPosition is about 1/4 of a turn (geared)
            feedForward = IntakeConstants.RETRACT_FEEDFORWARD_MAX_VOLTS
                *(retractedPositionDetected - this.slamInputs.positionRotations)/IntakeConstants.DEPLOYED_ROTATIONS;
            if (Math.abs(feedForward) > Math.abs(IntakeConstants.RETRACT_FEEDFORWARD_MAX_VOLTS)) {
                // clamp
                feedForward = Math.signum(feedForward)*Math.abs(IntakeConstants.RETRACT_FEEDFORWARD_MAX_VOLTS);
            }
        }

        double voltage = state.voltage
            - this.slamInputs.velocityRPS*IntakeConstants.DAMPENING
            + feedForward;

        this.slamIO.setVoltage(voltage);

        // run the rollers if deployed

        if (state == SlamState.DEPLOY_VOLTS) {
            this.rollersIO.setDutyCycle(IntakeConstants.ROLLERS_DUTY_CYCLE);
        }
        else {
            this.rollersIO.setDutyCycle(0.0);
        }
    }

    /**
     * Deploy the intake arms ans spin the rollers
     */
    public void deploy() {
        this.state = SlamState.DEPLOY_VOLTS;
    }

    /**
     * Deploy the intake arms and spin the rollers backwards
     */
    public void retract() {
        this.state = SlamState.RETRACT_VOLTS;
    }
}