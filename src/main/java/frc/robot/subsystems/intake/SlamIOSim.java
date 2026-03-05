package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.CONSTANTS.IntakeConstants;

public class SlamIOSim implements SlamIO {
    private final DCMotor armMotorModel = DCMotor.getNEO(1);

    private final DCMotorSim armLeftSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                armMotorModel,
                10,
                IntakeConstants.GEAR_RATIO_ARM),
                armMotorModel
            );

    private final DCMotorSim armRightSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                armMotorModel,
                2,
                IntakeConstants.GEAR_RATIO_ARM
                ),
                armMotorModel
            );
    
    public SlamIOSim() {
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        this.armLeftSim.update(0.02);
        this.armRightSim.update(0.02);
        inputs.velocityRPS = this.armLeftSim.getAngularVelocity().in(RotationsPerSecond);
        inputs.armLeftConnected = true;
        inputs.armRightConnected = true;
        inputs.armLeftAmps = this.armLeftSim.getCurrentDrawAmps();
        inputs.armRightAmps = this.armRightSim.getCurrentDrawAmps();
    }
    
    @Override
    public void setVoltage(double voltage) {
        this.armLeftSim.setInputVoltage(voltage);
        this.armRightSim.setInputVoltage(voltage);
    }
}
