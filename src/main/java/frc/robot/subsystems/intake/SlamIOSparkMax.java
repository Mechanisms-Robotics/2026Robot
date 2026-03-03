package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.CONSTANTS.IntakeConstants;

public class SlamIOSparkMax implements SlamIO {
    private final SparkMax armLeft = new SparkMax(IntakeConstants.ARM_ID_LEFT, MotorType.kBrushless);
    private final SparkMax armRight = new SparkMax(IntakeConstants.ARM_ID_RIGHT, MotorType.kBrushless);
    private final RelativeEncoder armLeftEncoder = this.armLeft.getEncoder();

    public SlamIOSparkMax() {
        var config = new SparkMaxConfig();
        config.follow(IntakeConstants.ARM_ID_LEFT, true);
        this.armRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.armLeft.configure(IntakeConstants.CONFIG_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double velocity = this.armLeftEncoder.getVelocity() / 60.0;
        inputs.velocityRPS = velocity;
    }

    @Override
    public void setVoltage(double voltage) {
        this.armLeft.setVoltage(voltage);
    }
}
