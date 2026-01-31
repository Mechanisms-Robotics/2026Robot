package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.CONSTANTS;
import edu.wpi.first.math.MathUtil;

/**
 * Simulation implementation of the feeder IO. Models a single NEO-like motor and exposes
 * the simulated values via the FeederIOInputs (AutoLog-friendly) so Advantage Scope will
 * display the running motor velocity/current/voltage.
 */
public class FeederIOSim implements FeederIO {

    private final FeederIOInputs inputs = new FeederIOInputs();

    // Simple motor simulation (1x NEO, 1:1 gearing). Tune inertia/gear ratio if you have real values.
    private static final DCMotor FEEDER_GEARBOX = DCMotor.getNEO(1);
    private static final double FEEDER_INERTIA = 0.001; // small inertia (kg*m^2), tune as needed
    private static final double FEEDER_GEAR_RATIO = 1.0;

    private final DCMotorSim motorSim;

    // Last applied volt/command
    private double motorCommand = 0.0; // [-1..1]
    private double appliedVolts = 0.0;

    public FeederIOSim() {
        // Build a simple motor plant using LinearSystemId to approximate motor behavior
        this.motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(FEEDER_GEARBOX, FEEDER_INERTIA, FEEDER_GEAR_RATIO),
            FEEDER_GEARBOX
        );
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // Compute applied volts from the last set command
        appliedVolts = MathUtil.clamp(motorCommand * 12.0, -12.0, 12.0);

        // Step the physics sim forward one robot loop
        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(CONSTANTS.ROBOT_LOOP_PERIOD);

        // Fill inputs for logging/Shuffleboard/Advantage scope
        inputs.feederConnected = true;
        inputs.feederOutputPercent = motorCommand;
        inputs.feederAppliedVolts = appliedVolts;
        inputs.feederCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());

        // motorSim reports angular velocity in rad/sec; convert to rotations/sec
        double omegaRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.feederVelocityRotationsPerSec = Units.radiansToRotations(omegaRadPerSec);

        // Simple jam detection: if current spikes very high, mark jam (tunable)
        inputs.jamDetected = inputs.feederCurrentAmps > 30.0;

        // Publish a few convenience entries to NetworkTables / Shuffleboard so they appear
        // live in Shuffleboard and therefore in Advantage Scope if you're monitoring NT.
        try {
            var table = NetworkTableInstance.getDefault().getTable("Feeder");
            table.getEntry("velocity_rps").setDouble(inputs.feederVelocityRotationsPerSec);
            table.getEntry("applied_volts").setDouble(inputs.feederAppliedVolts);
            table.getEntry("current_amps").setDouble(inputs.feederCurrentAmps);
            table.getEntry("output_percent").setDouble(inputs.feederOutputPercent);
        } catch (Exception e) {
            // Ignore NT errors in sim; Advantage Scope may not be running
        }
        // Optionally provide timestamped telemetry (not required by AutoLog, but useful)
        // Not implementing odometry here; just rely on AutoLog on the FeederIOInputs values.
    }

    @Override
    public void setMotorOpenLoop(double output) {
        motorCommand = Math.max(-1.0, Math.min(1.0, output));
    }

    @Override
    public void setMotorVelocity(double velocityRotPerSec) {
        // For the simple sim map velocity request to a percent command proportionally.
        // This is a crude mapping; replace with a controller if you need closed-loop sim.
        double percent = velocityRotPerSec / 10.0; // assume 10 RPS ==> 100% (tunable)
        motorCommand = Math.max(-1.0, Math.min(1.0, percent));
    }

    @Override
    public void stopMotor() {
        motorCommand = 0.0;
    }
}
