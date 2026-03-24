package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    private final DCMotor motorModel = DCMotor.getKrakenX60(2);
    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(motorModel, 1.0, 1.0), motorModel);

    private double kP = 0.3;
    private double kD = 0.01;

    private double desiredRpm = 0.0;
    private boolean closedLoop = false;
    
    public FlywheelIOSim() {}

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (this.closedLoop) {
            this.sim.setInputVoltage(
                (desiredRpm - this.sim.getAngularVelocityRPM()) * this.kP
                -this.sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * this.kD);
        }
        this.sim.update(0.2);

        inputs.rpm = this.sim.getAngularVelocityRPM();
        inputs.desiredRpm = this.desiredRpm;
    }

    @Override
    public void setVelocity(double rpm) {
        this.desiredRpm = rpm;
        this.closedLoop = true;
    }

    @Override
    public void setVoltage(double voltage) {
        this.sim.setInputVoltage(voltage);
        this.desiredRpm = 0.0;
        this.closedLoop = false;
    }
}
