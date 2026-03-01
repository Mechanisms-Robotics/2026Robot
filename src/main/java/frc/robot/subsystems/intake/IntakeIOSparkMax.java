package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax armLeft = new SparkMax(IntakeConstants.ARM_ID_LEFT, MotorType.kBrushless);
    private final SparkMax armRight = new SparkMax(IntakeConstants.ARM_ID_RIGHT, MotorType.kBrushless);
    private final SparkMax rollers = new SparkMax(IntakeConstants.ROLLERS_ID, MotorType.kBrushless);

    public IntakeIOSparkMax(SparkMax motor) {
        var config = new SparkMaxConfig();
        config.follow(IntakeConstants.ARM_ID_LEFT, true);
        armRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IntakeConstants.CONFIG_LEFT.
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        IntakeIO.super.updateInputs(inputs);
    }

    @Override
    public void setDeployed(boolean deployed) {
        IntakeIO.super.setDeployed(deployed);
    }

    @Override
    public void setSpeed(double dutyCycle) {
        rollers.set(dutyCycle);
    }

    private void setAngle(Rotation2d angle) {
        this.armLeft.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition);
    }
}
