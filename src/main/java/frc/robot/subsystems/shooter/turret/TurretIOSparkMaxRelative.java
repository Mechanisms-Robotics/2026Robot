package frc.robot.subsystems.shooter.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CONSTANTS.TurretConstants;

public class TurretIOSparkMaxRelative implements TurretIO {
    private final SparkMax motor = new SparkMax(20, MotorType.kBrushless);
    private final RelativeEncoder encoder = this.motor.getEncoder();

    public TurretIOSparkMaxRelative() {
        var config = new SparkMaxConfig();
        config.encoder
            .positionConversionFactor(TurretConstants.MOTOR_GEAR_RATIO)
            .velocityConversionFactor(TurretConstants.MOTOR_GEAR_RATIO);
        config.closedLoop
            .p(0.01)
            .positionWrappingEnabled(true)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        config.smartCurrentLimit(10);

        config.softLimit
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(-0.25)
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(0.25);

        this.encoder.setPosition(0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(this.encoder.getPosition());
        inputs.velocityRadiansPerSec = Units.rotationsToRadians(this.encoder.getVelocity());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.motor.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition);
    }

    @Override
    public void zero() {
        this.encoder.setPosition(0.0);
    }
}
