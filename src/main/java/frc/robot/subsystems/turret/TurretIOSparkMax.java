package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretIOSparkMax extends SubsystemBase {
    private final SparkMax motor = new SparkMax(20, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getAlternateEncoder();
    private final int SMART_CURRENT_LIMIT = 20;

    public TurretIOSparkMax() {
        var config = new SparkMaxConfig();
        config.absoluteEncoder
            .setSparkMaxDataPortConfig()
            .positionConversionFactor(1)
            .velocityConversionFactor(1)
            .inverted(false);

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SMART_CURRENT_LIMIT);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
