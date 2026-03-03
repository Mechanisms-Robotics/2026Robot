package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.CONSTANTS.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotor armMotorsModel = DCMotor.getNEO(2);
    private final DCMotor rollersModel = DCMotor.getNeo550(1);

    private final SingleJointedArmSim armSim = 
        new SingleJointedArmSim(armMotorsModel,
            IntakeConstants.GEAR_RATIO_ARM,
            1.0,
            Units.inchesToMeters(15.0),
            Units.degreesToRadians(IntakeConstants.DEPLOY_DEGREES),
            Units.degreesToRadians(IntakeConstants.START_DEGREES),
            false,
            Units.degreesToRadians(IntakeConstants.START_DEGREES)
        );
    
    private final FlywheelSim rollerSim = 
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                this.rollersModel,
                0.4,
                1.0
            ),
            this.rollersModel
        );

    private final double kP = 3.0;
    private double desiredRotations = IntakeConstants.START_DEGREES;

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        this.armSim.setInputVoltage(
            (this.desiredRotations - Units.radiansToRotations(this.armSim.getAngleRads())) * kP
        );
        this.armSim.update(0.2);
        this.rollerSim.update(0.2);

        inputs.armDegrees = Units.radiansToDegrees(this.armSim.getAngleRads());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.desiredRotations = angle.getRotations();
    }

    @Override
    public void deploy() {
        this.setAngle(Rotation2d.fromDegrees(IntakeConstants.DEPLOY_DEGREES));
    }

    @Override
    public void retract() {
        this.setAngle(Rotation2d.fromDegrees(IntakeConstants.RETRACT_DEGREES));
    }

    @Override
    public void setSpeed(double percent) {
        this.rollerSim.setInput(12.0 / percent);
    }
}
