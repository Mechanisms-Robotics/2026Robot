package frc.robot.subsystems.feeder;

/** Simple simulation implementation of FeederIO for local testing. */
public class FeederIOSim implements FeederIO {

    private final FeederIOInputs inputs = new FeederIOInputs();
    private double simulatedPosition = 0.0; // rotations
    private double motorCommand = 0.0; // last open-loop command [-1..1]

    public FeederIOSim() {
    }
    
    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // integrate a tiny amount per update to simulate motion
        simulatedPosition += motorCommand * 0.02; // ~0.02 rotations per cycle at full throttle (tunable)

        inputs.feederConnected = true;
        inputs.feederOutputPercent = motorCommand;
        inputs.feederAppliedVolts = motorCommand * 12.0;
        inputs.feederCurrentAmps = Math.abs(motorCommand) * 5.0; // simple proxy
        inputs.feederVelocityRotationsPerSec = motorCommand * 1.0; // proxy
        inputs.jamDetected = false;
    }

    @Override
    public void setMotorOpenLoop(double output) {
        motorCommand = Math.max(-1.0, Math.min(1.0, output));
    }

    @Override
    public void setMotorVelocity(double velocityRotPerSec) {
        // Map a velocity request to a percent for this simple sim
        motorCommand = Math.max(-1.0, Math.min(1.0, velocityRotPerSec));
    }

    @Override
    public void stopMotor() {
        motorCommand = 0.0;
    }
}
