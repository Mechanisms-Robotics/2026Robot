package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;

import org.littletonrobotics.junction.Logger;

/** High-level Feeder subsystem that owns an IO implementation. */
public class Feeder extends SubsystemBase {
    // Feeder has three motors; the IO layer provides per-motor inputs/outputs
    private final FeederIO kickFeederIO;
    private final FeederIO spindexerFeederIO;
    private final FeederIOInputsAutoLogged kickInputs = new FeederIOInputsAutoLogged();
    private final FeederIOInputsAutoLogged spindexerInputs = new FeederIOInputsAutoLogged();
    private double desiredVoltsKicker = 0.0;
    private double desiredVoltsSpindexer = 0.0;

    public Feeder(FeederIO kickFeederIO, FeederIO spindexerFeederIO) {
        this.kickFeederIO = kickFeederIO;
        this.spindexerFeederIO = spindexerFeederIO;
    }

    @Override
    public void periodic() {
        // Update hardware/sim inputs and publish to AdvantageKit
        this.kickFeederIO.updateInputs(this.kickInputs);
        this.spindexerFeederIO.updateInputs(this.spindexerInputs);

        Logger.processInputs("Feeder/Kicker", this.kickInputs);
        Logger.processInputs("Feeder/Spindexer", this.spindexerInputs);

        Logger.recordOutput("Feeder/Kicker/DesiredVolts", this.desiredVoltsKicker);
        Logger.recordOutput("Feeder/Spindexer/DesiredVolts", this.desiredVoltsSpindexer);


    }

    // Convenience passthroughs for commands
    // setMotorOpenLoop now takes volts (e.g. +/-12). The IO layer may apply to all motors or
    // split across motors depending on implementation.
    private void setMotorOpenLoop(double volts, FeederIO feederIO) {
        feederIO.setMotorOpenLoop(volts);
    }

    public void startFeeding() {
        this.desiredVoltsKicker = CONSTANTS.FEEDER_MOTOR_KICKER_VOLTAGE;
        this.desiredVoltsSpindexer = CONSTANTS.FEEDER_MOTOR_SPINDEXER_VOLTAGE;

        this.setMotorOpenLoop(this.desiredVoltsKicker, this.kickFeederIO);
        this.setMotorOpenLoop(this.desiredVoltsSpindexer, this.spindexerFeederIO);
    }

    public void stopFeeding() {
        this.kickFeederIO.stopMotor();
        this.spindexerFeederIO.stopMotor();
    }

    public void reverseFeeding() {
        this.desiredVoltsKicker = CONSTANTS.FEEDER_MOTOR_UNJAM_VOLTAGE;
        this.desiredVoltsSpindexer = CONSTANTS.FEEDER_MOTOR_UNJAM_VOLTAGE;
        
        this.setMotorOpenLoop(this.desiredVoltsKicker, this.kickFeederIO);
        this.setMotorOpenLoop(this.desiredVoltsSpindexer, this.spindexerFeederIO);
    }
}
