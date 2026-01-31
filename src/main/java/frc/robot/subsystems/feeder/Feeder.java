package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** High-level Feeder subsystem that owns an IO implementation. */
public class Feeder extends SubsystemBase {
    private final FeederIO feederIO;
    private final FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();

    public Feeder(FeederIO feederio) {
        this.feederIO = feederio;
    }

    @Override
    public void periodic() {
        // Update hardware/sim inputs and publish to AdvantageKit
        this.feederIO.updateInputs(this.inputs);
        // Record important values so they appear in AdvantageKit / Advantage Scope
        Logger.recordOutput("Feeder/OutputPercent", this.inputs.feederOutputPercent);
        Logger.recordOutput("Feeder/AppliedVolts", this.inputs.feederAppliedVolts);
        Logger.recordOutput("Feeder/CurrentAmps", this.inputs.feederCurrentAmps);
        Logger.recordOutput("Feeder/VelocityRps", this.inputs.feederVelocityRotationsPerSec);
    }

    // Convenience passthroughs for commands
    public void setMotorOpenLoop(double percent) {
        this.feederIO.setMotorOpenLoop(percent);
        System.out.println("Feeder set to " + percent);
    }

    public void setMotorVelocity(double velocityRps) {
        this.feederIO.setMotorVelocity(velocityRps);
    }

    public void stopMotor() {
        this.feederIO.stopMotor();
    }
}
