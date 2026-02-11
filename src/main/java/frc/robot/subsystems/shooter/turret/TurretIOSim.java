package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.LinearSystem;
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

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        motorSim.update(0.02);

        motorSim.setInputVoltage(
            MathUtil.clamp(this.appliedVoltage, -12, 12)
        );

        inputs.positionRadians = motorSim.getAngularPositionRad();
        inputs.velocityRadiansPerSec = motorSim.getAngularVelocityRadPerSec();
    }

    public void setPosition(double positionRadians) {
        this.appliedVoltage = (
            (positionRadians - motorSim.getAngularPositionRad()) * MOTOR_KP +
            (-motorSim.getAngularVelocityRadPerSec() * MOTOR_KD)
        );
    }
}
