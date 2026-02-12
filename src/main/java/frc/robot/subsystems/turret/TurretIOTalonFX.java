package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX motor;
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final PositionTorqueCurrentFOC positionCurrentRequest = new PositionTorqueCurrentFOC(0);
    private boolean useCurrentControl = false;

    public TurretIOTalonFX(TalonFXConfiguration config) {
        motor = new TalonFX(99);

        tryUntilOk(5, () -> 
            motor.getConfigurator().apply(config, 0.25)
        );
    }
    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        inputs.velocityRadiansPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    }
    
    @Override
    public void setPosition(double positionRadians) {
        double rotations = Units.radiansToRotations(positionRadians);
        if (this.useCurrentControl) {
            motor.setControl(positionCurrentRequest.withPosition(rotations));
        } else {
            motor.setControl(positionVoltageRequest.withPosition(rotations));
        }
    }

    @Override
    public void zero() {
        motor.setPosition(0);
    }
}
