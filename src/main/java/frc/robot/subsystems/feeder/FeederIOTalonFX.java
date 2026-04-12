package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Feeder IO implementation using CTRE TalonFX (Kraken) motors.
 * Controls three TalonFX devices and publishes per-motor inputs.
 */
public class FeederIOTalonFX implements FeederIO {

    private final TalonFX motor;


    // Status signals for motor 1
    private final StatusSignal<Voltage> motorVolts;
    private final StatusSignal<Current> motorCurrent;

    private final VoltageOut voltageRequest = new VoltageOut(0);


    /**
     * Construct with explicit CAN IDs for each motor.
     */
    public FeederIOTalonFX(int canId) {
        this.motor = new TalonFX(canId);


        // Create status signal handles
        this.motorVolts = this.motor.getMotorVoltage();
        this.motorCurrent = this.motor.getSupplyCurrent();

        // Configure reasonable periodic frame rates for these signals (reuse defaults if desired)
        BaseStatusSignal.setUpdateFrequencyForAll(
            20,
            this.motorVolts,
            this.motorCurrent
        );
    }


    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // Refresh signals in one shot (silently ignore exceptions)
        try {
            BaseStatusSignal.refreshAll(
                this.motorVolts,
                this.motorCurrent
            );

            inputs.feederConnected = true;

            // Motor 1
            inputs.feederAppliedVolts = this.motorVolts.getValueAsDouble();
            inputs.feederCurrentAmps = this.motorCurrent.getValueAsDouble();
            inputs.velocityRPM = this.motor.getVelocity().getValue().in(RPM);
           
           
            // Simple jam detection if any current is very high
            inputs.jamDetected = inputs.feederCurrentAmps > 40.0;
        } catch (Exception e) {
            // If reading signals fails, mark as disconnected to avoid silent bad telemetry
            inputs.feederConnected = false;
            inputs.feederAppliedVolts = Double.NaN;
            inputs.feederCurrentAmps = Double.NaN;
            inputs.jamDetected = false;
        }
    }

    @Override
    public void setMotorOpenLoop(double volts) {
        this.motor.setControl(this.voltageRequest.withOutput(volts));
    }

    @Override
    public void stopMotor() {
        this.motor.setControl(this.voltageRequest.withOutput(0.0));
    }
}
