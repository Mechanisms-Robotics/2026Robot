package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private ProfiledPIDController controller = new ProfiledPIDController(0.0, 0, 0.0, new Constraints(Math.PI, Math.PI * 4.0));
    private final double kCos = 0.1;
    
    public SlapIOSim() {
        SmartDashboard.putData("Intake/sim/controller", this.controller);
    }
    
    @Override
    public void updateInputs(SlapIOInputs inputs) {
        this.armLeftSim.update(0.02);
        this.armRightSim.update(0.02);

        inputs.leftConnected = true;
        inputs.rightConnected = true;
        inputs.velocityRadiansPerSecondLeft = this.armLeftSim.getAngularVelocity().in(DegreesPerSecond);
        inputs.positionDegreesLeft = this.armLeftSim.getAngularPosition().in(Degrees);
        inputs.currentAmps = this.armLeftSim.getCurrentDrawAmps();
        inputs.setpointDegrees = this.controller.getGoal().position / Math.PI * 180.0;

        double angleRadians = this.armLeftSim.getAngularPositionRad();
        double volts = this.kCos * Math.cos(angleRadians);

        this.armLeftSim.setInputVoltage(volts);
        this.armRightSim.setInputVoltage(volts);
    }
    
    @Override
    public void setAngle(Rotation2d angle) {
        this.controller.setGoal(angle.getRadians());
    }
}
