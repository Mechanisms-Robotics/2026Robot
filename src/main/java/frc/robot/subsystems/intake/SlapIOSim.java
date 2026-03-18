package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.CONSTANTS.IntakeConstants;

public class SlapIOSim implements SlapIO {
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

    private ProfiledPIDController controller = new ProfiledPIDController(1.0, 0, 0.5, new Constraints(3.0, 6.0));
    
    public SlapIOSim() {}
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        this.armLeftSim.update(1);
        this.armRightSim.update(1);

        inputs.leftConnected = true;
        inputs.rightConnected = true;
        inputs.velocityDegreesPerSecond = this.armLeftSim.getAngularVelocity().in(DegreesPerSecond);
        inputs.positionDegrees = this.armLeftSim.getAngularPosition().in(Degrees);
        inputs.currentAmps = this.armLeftSim.getCurrentDrawAmps();
        inputs.setPointDegrees = this.controller.getGoal().position * 360.0;

        double volts =
            this.controller.calculate(this.armLeftSim.getAngularPositionRotations());

        this.armLeftSim.setInputVoltage(volts);
        this.armRightSim.setInputVoltage(volts);
    }
    
    @Override
    public void setAngle(Rotation2d angle) {
        this.controller.setGoal(angle.getRotations());
    }
}
