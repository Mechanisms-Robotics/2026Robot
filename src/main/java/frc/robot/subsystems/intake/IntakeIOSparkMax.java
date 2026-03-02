package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.CONSTANTS.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax armLeft = new SparkMax(IntakeConstants.ARM_ID_LEFT, MotorType.kBrushless);
    private final SparkMax armRight = new SparkMax(IntakeConstants.ARM_ID_RIGHT, MotorType.kBrushless);
    private final RelativeEncoder armEncoder = this.armLeft.getEncoder();
    
    private final SparkMax rollers = new SparkMax(IntakeConstants.ROLLERS_ID, MotorType.kBrushless);

    public IntakeIOSparkMax() {
        var config = new SparkMaxConfig();
        config.follow(IntakeConstants.ARM_ID_LEFT, true);
        this.armRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.armLeft.configure(IntakeConstants.CONFIG_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.armEncoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.armDegrees = this.getAngle().getDegrees();
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
    public void setSpeed(double dutyCycle) {
        this.rollers.set(dutyCycle);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.armLeft.getClosedLoopController().setSetpoint(angle.rotateBy(Rotation2d.fromDegrees(IntakeConstants.START_DEGREES)).getRotations(), ControlType.kPosition);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(this.armEncoder.getPosition());
    }
}
