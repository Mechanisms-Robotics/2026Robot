package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.CONSTANTS;
import edu.wpi.first.math.MathUtil;

/**
 * Simulation implementation of the feeder IO. Models a TalonFX / Falcon500 (Kraken)
 * motor and exposes the simulated values via the FeederIOInputs (AutoLog-friendly)
 * so Advantage Scope will display the running motor velocity/current/voltage.
 */
public class FeederIOSim implements FeederIO {
    

    // Simple motor simulation. Use a Falcon500/TalonFX-equivalent motor model.
    // Tune inertia/gear ratio if you have real values.
    private static final DCMotor FEEDER_GEARBOX = DCMotor.getFalcon500(1);
    private static final double FEEDER_INERTIA = 0.001; // small inertia (kg*m^2), tune as needed
    private static final double FEEDER_GEAR_RATIO = 1.0;

    private final DCMotorSim motorSim;
    

    // Last applied volts for each motor
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
    // Step the physics sims forward one robot loop using their per-motor applied volts
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(CONSTANTS.ROBOT_LOOP_PERIOD);


        // Fill inputs for logging/Shuffleboard/Advantage scope
        inputs.feederConnected = true;
    // Motor 1 values
    inputs.feederAppliedVolts = appliedVolts;
    inputs.feederCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());

        // Simple jam detection: if current spikes very high, mark jam (tunable)
    // Simple jam detection: if any motor current spikes very high, mark jam (tunable)
    inputs.jamDetected = inputs.feederCurrentAmps > 30.0;
        // Publish a few convenience entries to NetworkTables / Shuffleboard so they appear
        // live in Shuffleboard and therefore in Advantage Scope if you're monitoring NT.
        try {
            var table = NetworkTableInstance.getDefault().getTable("Feeder");
            table.getEntry("applied_volts").setDouble(inputs.feederAppliedVolts);
            table.getEntry("current_amps").setDouble(inputs.feederCurrentAmps);
        } catch (Exception e) {
            // Ignore NT errors in sim; Advantage Scope may not be running
        }
        // Optionally provide timestamped telemetry (not required by AutoLog, but useful)
        // Not implementing odometry here; just rely on AutoLog on the FeederIOInputs values.
    }


    @Override
    public void setMotorOpenLoop(double volts) {
        this.motorSim.setInputVoltage(volts);
        this.motorSim.update(CONSTANTS.ROBOT_LOOP_PERIOD);
    }

    @Override
    public void stopMotor() {
        this.motorSim.setInputVoltage(0.0);
        this.motorSim.update(CONSTANTS.ROBOT_LOOP_PERIOD);
    }
}
