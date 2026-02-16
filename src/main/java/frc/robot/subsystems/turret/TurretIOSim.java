package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private final double MOTOR_KP = 2;
    private final double MOTOR_KD = 0.0;
    private final DCMotor GEARBOX = DCMotor.getFalcon500(1);
    private final DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 100),
        GEARBOX);
    private double appliedVoltage = 0;
    // private double motorOffset = 0.0;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        this.motorSim.update(0.02);

        this.motorSim.setInputVoltage(
            MathUtil.clamp(this.appliedVoltage, -12, 12)
        );

        inputs.positionRadians = this.motorSim.getAngularPositionRad();
        inputs.velocityRadiansPerSec = this.motorSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void setPosition(Rotation2d position) {
        double radians = position.getRadians();
        this.appliedVoltage = (
            (radians - this.motorSim.getAngularPositionRad()) * MOTOR_KP +
            (-this.motorSim.getAngularVelocityRadPerSec() * MOTOR_KD)
        );
    }

    @Override
    public void zero() {
        // this.motorOffset = this.motorSim.getAngularPositionRad();
    }
}
