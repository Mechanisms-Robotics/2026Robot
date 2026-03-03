package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);
        Logger.processInputs("Intake", this.inputs);
    }

    /**
     * Deploy the intake arms ans spin the rollers
     */
    public void intake() {
        Logger.recordOutput("Intake/state", "intaking");
        io.deploy();
        io.setSpeed(IntakeConstants.INTAKE_DUTY_CYCLE);
    }

    /**
     * Deploy the intake arms and spin the rollers backwards
     */
    public void outtake() {
        Logger.recordOutput("Intake/state", "outtaking");
        io.deploy();
        io.setSpeed(IntakeConstants.OUTTAKE_DUTY_CYCLE);
    }

    /**
     * Retract the intake arms and stop the rollers
     */
    public void retract() {
        Logger.recordOutput("Intake/state", "retracted");
        io.retract();
        io.setSpeed(0.0);
    }
}