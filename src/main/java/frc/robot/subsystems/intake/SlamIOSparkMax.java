package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.CONSTANTS.IntakeConstants;

public class SlamIOSparkMax implements SlamIO {
    private final SparkMax armLeft = new SparkMax(IntakeConstants.ARM_ID_LEFT, MotorType.kBrushless);
    private final SparkMax armRight = new SparkMax(IntakeConstants.ARM_ID_RIGHT, MotorType.kBrushless);
    private final RelativeEncoder armLeftEncoder = this.armLeft.getEncoder();

    public SlamIOSparkMax() {
        var config_right = new SparkMaxConfig();
        config_right.follow(IntakeConstants.ARM_ID_LEFT, true);
        config_right.idleMode(IdleMode.kBrake);
        
        this.armRight.configure(config_right, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.armLeft.configure(IntakeConstants.CONFIG_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRPS = this.armLeftEncoder.getVelocity() / 60.0;
        inputs.positionRotations = this.armLeftEncoder.getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        this.armLeft.setVoltage(voltage);
    }
}
