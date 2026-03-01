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
    private boolean stopPower = false;
    
    public FlywheelIOSim() {}

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        this.sim.setInputVoltage(this.stopPower ? 0.0 :
            (desiredRpm - this.sim.getAngularVelocityRPM()) * this.kP
            -this.sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * this.kD);
        this.sim.update(0.2);

        inputs.rpm = this.sim.getAngularVelocityRPM();
    }

    @Override
    public void setVelocity(double rpm) {
        this.desiredRpm = rpm;
        this.stopPower = false;
    }

    @Override
    public void stopPower() {
        this.stopPower = true;
    }
}
