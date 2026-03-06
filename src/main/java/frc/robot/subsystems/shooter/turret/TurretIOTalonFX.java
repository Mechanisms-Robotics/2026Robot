package frc.robot.subsystems.shooter.turret;

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
    private double desiredRotations = 0.0;
    private double velocity = 0.0;

    public TurretIOTalonFX(TalonFXConfiguration config) {
        motor = new TalonFX(20);

        tryUntilOk(5, () -> 
            motor.getConfigurator().apply(config, 0.25)
        );
    }
    
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        this.currentMotorRotations = this.motor.getPosition().getValueAsDouble();
        this.velocity = this.motor.getVelocity().getValueAsDouble();

        inputs.positionDegrees = Units.rotationsToRadians(this.currentMotorRotations);
        inputs.velocityDegreesPerSecond = Units.rotationsToRadians(this.velocity);
        inputs.current = this.motor.getTorqueCurrent().getValueAsDouble();
        inputs.connected = this.motor.isConnected();

        // The shortest possible rotation for the turret to move, which if done may break the wire chain
        Rotation2d relative = Rotation2d.fromRotations(desiredRotations).relativeTo(Rotation2d.fromRotations(this.currentMotorRotations));
        // Error in the PID sence
        double error = relative.getRotations();
        // The desired position including the number of times the turret has made a revolution
        double target = Units.rotationsToRadians(this.currentMotorRotations) + relative.getRadians();

        /* If the absolute desired position is past the limits of the wire chain,
           go a different direction to the desired position; flip around the other way. */
        if (target > this.forwardLimit || target < this.reverseLimit) {
            /* Changes direction by reversing the magnitude.
               Derivation:
               The magnitude of the error is 1 rotation minus the absolute value of the error:
                    1 - |error|
               And the direction of the error is the opposite of the signum (sign) of the error
                    -signum(error)
               Our flipped error becomes:
               -(1 - |error|) * signum(error)
               We can simplify:
               = -(signum(error) - error)
               = error - signum(error)
               Which can be simplified with -=
            */
            error -= Math.signum(error);
        }
        // Use PD controller with voltage controll
        this.motor.setVoltage(MathUtil.clamp(
            error * kP - this.velocity * kD, -maxVoltage, maxVoltage
        ));
    }
    
    @Override
    public void setAngle(Rotation2d position) {
        this.desiredRotations = position.getRotations();
    }

    @Override
    public void zero() {
        this.motor.setPosition(0);
    }
}
