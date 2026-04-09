package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Fahrenheit;
// -340 30
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.util.PhoenixUtil;

public class RollersIOTalonFX implements RollersIO {
    private final TalonFX motor = new TalonFX(IntakeConstants.ROLLERS_CAN_ID);

    public RollersIOTalonFX() {
        PhoenixUtil.tryUntilOk(5, () -> this.motor.getConfigurator().apply(IntakeConstants.CONFIG_ROLLERS_TALON));
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        inputs.currentAmps = this.motor.getSupplyCurrent().getValueAsDouble();
        inputs.isConnected = this.motor.isConnected();
        inputs.tempFahrenheit = this.motor.getDeviceTemp().getValue().in(Fahrenheit);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        this.motor.set(dutyCycle);
    }
}
