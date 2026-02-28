package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.CONSTANTS.HoodConstants;

public class HoodIOSim implements HoodIO {
    private final DCMotor motorModel = DCMotor.getKrakenX44(1);
    private final SingleJointedArmSim sim =
        new SingleJointedArmSim(
            motorModel,
            HoodConstants.CONFIG.Feedback.SensorToMechanismRatio,
            0.05,
            0.2,
            Units.degreesToRadians(22.0),
            Units.degreesToRadians(52.0),
            false,
            Units.degreesToRadians(22.0)
        );
    private final double kP = 1.0;
    private final double kD = 0.0;
    
    private double desiredRadians = 22.0;
    
    public HoodIOSim() {}

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.setInputVoltage(
            (this.desiredRadians - sim.getAngleRads()) * this.kP
            -sim.getVelocityRadPerSec() * kD
        );
        sim.update(0.2);

        inputs.positionRadians = sim.getAngleRads();
    }

    @Override
    public void setPosition(double positionRadians) {
        this.desiredRadians = positionRadians;
    }
}
