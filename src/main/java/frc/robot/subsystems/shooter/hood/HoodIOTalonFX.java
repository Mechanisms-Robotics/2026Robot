package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class HoodIOTalonFX implements HoodIO {
    // Kraken X44
    private final TalonFX motor = new TalonFX(23);
    // REV throughbore encoder
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(3);
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);
    private final double ENCODER_HOOD_RATIO = 1.0/9.412;
    public HoodIOTalonFX() {
        var config = new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(48.0/12.0 * ENCODER_HOOD_RATIO)

            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(20))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKD(0.0)
            );
        this.motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(getPosition());
    }

    @Override
    public void setPosition(double positionRadians) {
        double rotations = Units.radiansToRotations(positionRadians);
        double distanceToTarget = rotations - getPosition();
        /* Since the motor uses a relative encoder, and the position request uses the motor's
           encoder, calculate the position the motor needs to be at for position request. */
        double target = this.motor.getPosition().getValueAsDouble() + distanceToTarget;
        this.motor.setControl(positionRequest.withPosition(target));
    }

    private double getPosition() {
        return this.encoder.get() * ENCODER_HOOD_RATIO;
    }
}
