package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.CONSTANTS.HoodConstants;
import frc.robot.CONSTANTS.TurretConstants;

public class HoodIOSim implements HoodIO {
    private final DCMotor motorModel = DCMotor.getKrakenX44(1);
    private final SingleJointedArmSim sim =
        new SingleJointedArmSim(
            motorModel,
            HoodConstants.CONFIG.Feedback.SensorToMechanismRatio,
            0.04,
            0.2,
            Units.degreesToRadians(HoodConstants.MIN_DEGREES),
            Units.degreesToRadians(HoodConstants.MAX_DEGREES),
            false,
            Units.degreesToRadians(22.0)
        );
    private final double kP = 0.2;
    private final double kD = 0.4;
    
    private double desiredRadians = TurretConstants.MIN_DEGREES / 180.0 * Math.PI;
    
    public HoodIOSim() {
        this.sim.setState(TurretConstants.MIN_DEGREES / 180.0 * Math.PI, 0.0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double appliedVolts = (this.desiredRadians - this.sim.getAngleRads()) * this.kP
            -this.sim.getVelocityRadPerSec() * kD;
        this.sim.setInputVoltage(
            (this.desiredRadians - this.sim.getAngleRads()) * this.kP
            -this.sim.getVelocityRadPerSec() * kD
        );
        this.sim.update(0.2);

        inputs.positionDegrees = this.sim.getAngleRads() / Math.PI * 180.0;
        inputs.setpointDegrees = this.desiredRadians / Math.PI * 180.0;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = this.sim.getCurrentDrawAmps();
        inputs.connected = true;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.desiredRadians = angle.getRadians();
    }
}
