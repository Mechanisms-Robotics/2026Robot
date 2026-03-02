package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CONSTANTS.HoodConstants;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
    // Kraken X44
    private final TalonFX motor = new TalonFX(23);

    private double desiredRadians = Units.degreesToRadians(HoodConstants.MIN_DEGREES);
    
    public HoodIOTalonFX() {
        PhoenixUtil.tryUntilOk(5, () -> this.motor.getConfigurator().apply(HoodConstants.CONFIG));
        PhoenixUtil.tryUntilOk(5, () -> this.motor.setPosition(0.0)); // set to zero rotations
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double position = this.getPosition();
        Logger.recordOutput("Hood/desiredTest", this.desiredRadians);

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
        return Units.rotationsToRadians(this.motor.getPosition().getValueAsDouble())  +
            Units.degreesToRadians(HoodConstants.MIN_DEGREES);
    }
}
