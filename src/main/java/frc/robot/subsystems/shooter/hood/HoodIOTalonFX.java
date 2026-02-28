package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.CONSTANTS.HoodConstants;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
    // Kraken X44
    private final TalonFX motor = new TalonFX(23);
    // REV throughbore encoder
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(3);

    private double desiredRadians = 0.0;
    
    public HoodIOTalonFX() {
        PhoenixUtil.tryUntilOk(5, () -> this.motor.getConfigurator().apply(HoodConstants.CONFIG));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(getPosition());
        
        this.motor.setVoltage(
            (this.desiredRadians - Units.rotationsToRadians(getPosition())) * HoodConstants.kP +
            -this.motor.getVelocity().getValueAsDouble() * HoodConstants.kD
        );
    }

    @Override
    public void setPosition(double positionRadians) {
        this.desiredRadians = MathUtil.clamp(positionRadians, HoodConstants.MIN_DEGREES, HoodConstants.MAX_DEGREES);
    }

    private double getPosition() {
        return this.encoder.get() * HoodConstants.ENCODER_HOOD_RATIO;
    }
}
