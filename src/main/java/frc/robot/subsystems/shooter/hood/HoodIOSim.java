package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Timeouts.HoodConstants;

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
    
    private double desiredRadians = 22.0;
    
    public HoodIOSim() {}

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        this.sim.setInputVoltage(
            (this.desiredRadians - this.sim.getAngleRads()) * this.kP
            -this.sim.getVelocityRadPerSec() * kD
        );
        this.sim.update(0.2);

        inputs.positionDegrees = Units.radiansToDegrees(this.sim.getAngleRads());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.desiredRadians = angle.getRadians();
    }
}
