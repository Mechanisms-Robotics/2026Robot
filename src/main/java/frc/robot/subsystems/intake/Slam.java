package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.CONSTANTS.IntakeConstants.SlamState;

public class Slam extends SubsystemBase {
    private final SlamIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    SlamState state = SlamState.RETRACT;

    public Slam(SlamIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Intake", this.inputs);

        double voltage = state.voltage - this.inputs.velocityRPS * IntakeConstants.kD;
        this.io.setVoltage(voltage);
    }

    /**
     * Deploy the intake arms ans spin the rollers
     */
    public void deploy() {
        this.state = SlamState.DEPLOY;
    }

    /**
     * Deploy the intake arms and spin the rollers backwards
     */
    public void retract() {
        this.state = SlamState.RETRACT;
    }
}