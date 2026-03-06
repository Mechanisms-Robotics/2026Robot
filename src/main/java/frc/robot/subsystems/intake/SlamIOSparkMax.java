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
    private final SparkMax armLeft = new SparkMax(IntakeConstants.ARM_CAN_ID_LEFT, MotorType.kBrushless);
    private final SparkMax armRight = new SparkMax(IntakeConstants.ARM_CAN_ID_RIGHT, MotorType.kBrushless);
    private final RelativeEncoder armLeftEncoder = this.armLeft.getEncoder();

    public SlamIOSparkMax() {
        var config_right = new SparkMaxConfig();
        config_right.follow(IntakeConstants.ARM_CAN_ID_LEFT, true);
        config_right.idleMode(IdleMode.kBrake);

        // Configure the leader first, then the follower. Some firmware versions
        // apply follower settings better when the leader is configured first.
        this.armLeft.configure(IntakeConstants.CONFIG_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.armRight.configure(config_right, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double lastAppliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRPS = this.armLeftEncoder.getVelocity() / 60.0;
        inputs.positionRotations = this.armLeftEncoder.getPosition();
        inputs.appliedVolts = this.lastAppliedVolts;
    }

    @Override
    public void setVoltage(double voltage) {
        this.armLeft.setVoltage(voltage);
        this.lastAppliedVolts = voltage;
    }
}
