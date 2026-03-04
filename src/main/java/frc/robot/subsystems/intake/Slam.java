package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.CONSTANTS.IntakeConstants.SlamState;

public class Slam extends SubsystemBase {
    private final SlamIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    SlamState state = SlamState.RETRACT_VOLTS;

    // the way our motors are configured the positive is the retracted direction
    private double retractedPositionDetected = Double.MIN_VALUE;

    public Slam(SlamIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Intake", this.inputs);

        // Capture the most retracted position. If we accidentally start
        // with the intake out it's okay. It probably won't retract until
        // we manually close it and capture the new retracted position.
        if (this.inputs.positionRotations > retractedPositionDetected) {
            retractedPositionDetected = this.inputs.positionRotations;
        }

        // use voltage control but dampen speed

        double feedForward = 0.0;

        if (state == SlamState.RETRACT_VOLTS) { // only use feedforward on retraction
            // the deployedPosition is about 1/4 of a turn (geared)
            final double DEPLOYED_ROTATIONS = 0.25; // estimated
            feedForward = IntakeConstants.RETRACT_FEEDFORWARD_MAX_VOLTS
                *(retractedPositionDetected - this.inputs.positionRotations)/DEPLOYED_ROTATIONS;
            if (Math.abs(feedForward) > Math.abs(IntakeConstants.RETRACT_FEEDFORWARD_MAX_VOLTS)) {
                // clamp
                feedForward = Math.signum(feedForward)*Math.abs(IntakeConstants.RETRACT_FEEDFORWARD_MAX_VOLTS);
            }
        }

        double voltage = state.voltage
            - this.inputs.velocityRPS*IntakeConstants.DAMPENING
            + feedForward;
            
        this.io.setVoltage(voltage);
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