package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

    }

    public void intake() {
        io.setDeployed(true);
        io.setSpeed(CONSTANTS.IntakeConstants.INTAKE_DUTY_CYCLE);
    }

    public void outtake() {
        io.setDeployed(true);
        io.setSpeed(CONSTANTS.IntakeConstants.OUTTAKE_DUTY_CYCLE);
    }

    public void retract() {
        // retracts and stops the intake
        io.setDeployed(false);
        io.setSpeed(0.0);
    }
}