package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.CONSTANTS;
import edu.wpi.first.math.MathUtil;

public class ClimberIOSim implements ClimberIO{
    
    private final ClimberIOInputs inputs = new ClimberIOInputs();

    private static final DCMotor CLIMBER_GEARBOX = DCMotor.getNEO(1);
    private static final double CLIMBER_INERTIA = 0.001;
    private final DCMotorSim motorSim;

    private double motorCommand = 0.0;
    private double appliedVolts = 0.0;

    public ClimberIOSim() {
        this.motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(CLIMBER_GEARBOX, CLIMBER_INERTIA, CONSTANTS.CLIMBER_GEAR_RATIO),
            CLIMBER_GEARBOX
        );
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        appliedVolts = MathUtil.clamp(motorCommand * 12.0, -12.0, 12.0);

        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(CONSTANTS.ROBOT_LOOP_PERIOD);

        inputs.climberConnected = true;
        inputs.climberOutputPercent = motorCommand;
        inputs.climberAppliedVolts = appliedVolts;
        inputs.climberCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());

        double omegaRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.climberVelocityRotationsPerSec = Units.radiansToRotations(omegaRadPerSec);

        inputs.jamDetected = inputs.climberCurrentAmps > 30.0;

        try {
            var table = NetworkTableInstance.getDefault().getTable("Climber");
            table.getEntry("velocity_rps").setDouble(inputs.climberVelocityRotationsPerSec);
            table.getEntry("applied_volts").setDouble(inputs.climberAppliedVolts);
            table.getEntry("current_amps").setDouble(inputs.climberCurrentAmps);
            table.getEntry("output_percent").setDouble(inputs.climberOutputPercent);
        } catch (Exception e) {
            //Ignore
        }

    }

    @Override
    public void setMotorOpenLoop(double output) {
        motorCommand = Math.max(-1.0, Math.min(1.0, output));
    }

    @Override
    public void setMotorVelocity(double velocityRotPerSec) {
        double percent = velocityRotPerSec / 10.0;
        motorCommand = Math.max(-1.0, Math.min(1.0, percent));
    }

    @Override
    public void stopMotor() { 
        motorCommand = 0.0;
    }
}
