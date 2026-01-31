package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

/** Feeder IO implementation using a REV CANSparkMax (Neo). */
public class FeederIONeo implements FeederIO {
    private final FeederIOInputs inputs = new FeederIOInputs();

    // Motor controller and helpers
    private final SparkMax feederMotor;
    private final RelativeEncoder encoder;

    /**
     * Construct a feeder IO instance using the provided CAN ID.
     *
     * @param canId CAN ID of the Neo/SparkMax
     */
    public FeederIONeo(int canId) {
        this.feederMotor = new SparkMax(canId, MotorType.kBrushless);
        this.encoder = this.feederMotor.getEncoder();

    }

    /**
     * Convenience constructor. Replace the default CAN ID with your hardware value or
     * call the (int) constructor from RobotContainer when wiring hardware.
     */
    public FeederIONeo() {
        this(10); // TODO: replace 10 with your actual feeder CAN ID or instantiate with explicit id
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // Connection: Spark Max doesn't provide an "isConnected" API; assume true if controller exists
        inputs.feederConnected = true;

        // Open-loop output percent (last set value)
        try {
            inputs.feederOutputPercent = this.feederMotor.get();
        } catch (Exception e) {
            inputs.feederOutputPercent = Double.NaN;
        }

        // Applied volts (approximate): busVoltage * appliedPercent
        double busVolt = Double.NaN;
        try {
            busVolt = this.feederMotor.getBusVoltage();
        } catch (Exception e) {
            busVolt = Double.NaN;
        }
        if (!Double.isNaN(busVolt) && !Double.isNaN(inputs.feederOutputPercent)) {
            inputs.feederAppliedVolts = busVolt * inputs.feederOutputPercent;
        } else if (!Double.isNaN(inputs.feederOutputPercent)) {
            inputs.feederAppliedVolts = inputs.feederOutputPercent * 12.0;
        } else {
            inputs.feederAppliedVolts = Double.NaN;
        }

        // Current (amps)
        try {
            inputs.feederCurrentAmps = this.feederMotor.getOutputCurrent();
        } catch (Exception e) {
            inputs.feederCurrentAmps = Double.NaN;
        }

        // Velocity: SparkMax encoder returns RPM by default; convert to rotations/sec
        try {
            double rpm = this.encoder.getVelocity();
            inputs.feederVelocityRotationsPerSec = rpm / 60.0;
        } catch (Exception e) {
            inputs.feederVelocityRotationsPerSec = Double.NaN;
        }

        // Jam detection is higher-level — leave false for now
        inputs.jamDetected = false;
    }

    @Override
    public void setMotorOpenLoop(double output) {
        // Clamp and set open-loop percent
        double clamped = Math.max(-1.0, Math.min(1.0, output));
        this.feederMotor.set(clamped);
    }

    @Override
    public void setMotorVelocity(double velocityRotPerSec) {
        // Use Spark Max closed-loop velocity if PID is configured. SparkMax velocity uses RPM units.
        double targetRpm = velocityRotPerSec * 60.0;
    }

    @Override
    public void stopMotor() {
        this.feederMotor.set(0.0);
    }
}
