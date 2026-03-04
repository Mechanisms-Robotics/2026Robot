package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private final double MOTOR_KP = 5;
    private final double MOTOR_KD = 0.0;
    private final DCMotor GEARBOX = DCMotor.getFalcon500(1);
    private final DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 100),
        GEARBOX);
    private double desiredRadians = 0.0;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        this.motorSim.update(0.02);

        this.motorSim.setInputVoltage(
            MathUtil.clamp(
                (desiredRadians - this.motorSim.getAngularPositionRad()) * MOTOR_KP +
                (-this.motorSim.getAngularVelocityRadPerSec() * MOTOR_KD)
            , -12, 12)
        );

        inputs.positionDegrees = this.motorSim.getAngularPosition().in(Degrees);
        inputs.velocityDegreesPerSecond = this.motorSim.getAngularVelocity().in(DegreesPerSecond);
        inputs.current = this.motorSim.getCurrentDrawAmps();
        inputs.connected = true;
    }

    @Override
    public void setAngle(Rotation2d position) {
        this.desiredRadians = position.getRadians();
    }

    @Override
    public void zero() {
    }
}
