package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX motor;
    private final double maxVoltage = 0.5;
    private final double kP = 4.0;
    private final double kD = 0.05;
    private final double forwardLimit = 3.2;
    private final double reverseLimit = -3.2;
    private double currentMotorRotations = 0.0;
    private double velocity = 0.0;

    public TurretIOTalonFX(TalonFXConfiguration config) {
        motor = new TalonFX(20);

        tryUntilOk(5, () -> 
            motor.getConfigurator().apply(config, 0.25)
        );
    }
    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        this.currentMotorRotations = motor.getPosition().getValueAsDouble();
        this.velocity = motor.getVelocity().getValueAsDouble();

        inputs.positionRadians = Units.rotationsToRadians(this.currentMotorRotations);
        inputs.velocityRadiansPerSec = Units.rotationsToRadians(this.velocity);
    }
    
    @Override
    public void setPosition(Rotation2d position) {
        Rotation2d relative = position.relativeTo(Rotation2d.fromRotations(this.currentMotorRotations));
        double error = relative.getRotations();
        double target = Units.rotationsToRadians(this.currentMotorRotations) + relative.getRadians();
        if (target > forwardLimit || target < reverseLimit) {
            error -= Math.signum(error);
        }
        motor.setVoltage(MathUtil.clamp(
            error * kP - this.velocity * kD, -maxVoltage, maxVoltage
        ));
    }

    @Override
    public void zero() {
        motor.setPosition(0);
    }
}
