package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.CONSTANTS.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotor armMotorModel = DCMotor.getNEO(1);
    private final DCMotor rollersModel = DCMotor.getNeo550(1);

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
    
    private final FlywheelSim rollerSim = 
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                this.rollersModel,
                0.5,
                IntakeConstants.GEAR_RATIO_ARM
            ),
            this.rollersModel
        );

    private final double kP = 20.0;
    private final double kD = 0.35;
    private double desiredRotations = Units.degreesToRotations(IntakeConstants.START_DEGREES);

    public IntakeIOSim() {
        this.armLeftSim.setAngle(Units.degreesToRadians(IntakeConstants.START_DEGREES));
        this.armRightSim.setAngle(Units.degreesToRadians(IntakeConstants.START_DEGREES));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double voltage = (this.desiredRotations - this.armLeftSim.getAngularPosition().in(Rotations)) * this.kP
         - this.armLeftSim.getAngularVelocity().in(DegreesPerSecond) * this.kD;
        double voltage2 = (this.desiredRotations - this.armLeftSim.getAngularPosition().in(Rotations)) * this.kP
         - this.armLeftSim.getAngularVelocity().in(DegreesPerSecond) * this.kD;
         
        this.armLeftSim.setInputVoltage(
            voltage
        );
        this.armRightSim.setInputVoltage(
            voltage2
        );

        this.armLeftSim.update(0.2);
        this.armRightSim.update(0.2);
        this.rollerSim.update(0.2);

        inputs.armLeftDegrees = this.armLeftSim.getAngularPosition().in(Degrees);
        inputs.armRightDegrees = this.armRightSim.getAngularPosition().in(Degrees);
        inputs.desiredDegrees = Units.rotationsToDegrees(this.desiredRotations);
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
