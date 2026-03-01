package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.CONSTANTS.HoodConstants;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
    // Kraken X44
    private final TalonFX motor = new TalonFX(23);
    // REV throughbore encoder
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    private double desiredRadians = this.getPosition();
    
    public HoodIOTalonFX() {
        PhoenixUtil.tryUntilOk(5, () -> this.motor.getConfigurator().apply(HoodConstants.CONFIG));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double position = this.getPosition();

        inputs.positionDegrees = Units.radiansToDegrees(position);
        
        this.motor.setVoltage(
            (this.desiredRadians - position) * HoodConstants.kP
        );
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.desiredRadians = MathUtil.clamp(angle.getRadians(), Units.degreesToRadians(HoodConstants.MIN_DEGREES), Units.degreesToRadians(HoodConstants.MAX_DEGREES));
    }

    /**
     * Returns the position of the hood, in radians.
     */
    private double getPosition() {
        return Units.rotationsToRadians(this.encoder.get() * HoodConstants.ENCODER_HOOD_RATIO) 
            + HoodConstants.HOOD_OFFSET_RADIANS;
    }
}
