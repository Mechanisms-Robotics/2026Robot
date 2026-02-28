package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    private final DCMotor motorModel = DCMotor.getKrakenX60(2);
    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(motorModel, 1.0, 1.0), motorModel);

    private double desiredRps = 0.0;
    private double kP = 1.0;
    private double kD = 0.0;
    
    public FlywheelIOSim() {}

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        this.sim.setInputVoltage(
            (desiredRps - this.sim.getAngularVelocityRPM() / 60.0) * this.kP
            -this.sim.getAngularAccelerationRadPerSecSq() * this.kD);
        this.sim.update(0.2);

        inputs.rps = this.sim.getAngularVelocityRPM() / 60.0;
    }

    @Override
    public void setVelocity(double rps) {
        this.desiredRps = rps;
    }
}
