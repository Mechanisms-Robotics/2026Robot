package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.CONSTANTS.IntakeConstants;

public class SlapIOSparkMax implements SlapIO {
    private final SparkMax armLeft = new SparkMax(IntakeConstants.ARM_CAN_ID_LEFT, MotorType.kBrushless);
    private final SparkMax armRight = new SparkMax(IntakeConstants.ARM_CAN_ID_RIGHT, MotorType.kBrushless);
    private final RelativeEncoder armLeftEncoder = this.armLeft.getEncoder();

    public SlapIOSparkMax() {
        var config_right = new SparkMaxConfig();
        config_right.follow(IntakeConstants.ARM_CAN_ID_LEFT, true);
        config_right.idleMode(IdleMode.kBrake);

        // Configure the leader first, then the follower. Some firmware versions
        // apply follower settings better when the leader is configured first.
        this.armLeft.configure(IntakeConstants.CONFIG_LEFT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.armRight.configure(config_right, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.armLeftEncoder.setPosition(IntakeConstants.START_ANGLE.getRotations());
    }


    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityDegreesPerSecond = this.armLeftEncoder.getVelocity() / 60.0 * 360.0; // rpm -> rps -> degrees/s
        inputs.positionDegrees = this.armLeftEncoder.getPosition() * 360.0;
        inputs.currentAmps = this.armLeft.getOutputCurrent();
        inputs.leftConnected = this.armLeft.getLastError() == REVLibError.kOk;
        inputs.rightConnected = this.armRight.getLastError() == REVLibError.kOk;
        inputs.setPointDegrees = this.armLeft.getClosedLoopController().getSetpoint() * 360.0;
    }

    public void setAngle(Rotation2d angle) {
        this.armLeft.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition);
        this.armRight.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition);
    }
}
