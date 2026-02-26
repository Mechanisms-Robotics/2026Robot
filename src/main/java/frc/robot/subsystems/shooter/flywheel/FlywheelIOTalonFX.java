package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final int LEADER_ID = 21;
    private final TalonFX leader = new TalonFX(LEADER_ID);
    private final TalonFX follower = new TalonFX(22);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    public FlywheelIOTalonFX() {
        var config = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.leader.getConfigurator().apply(config);

        this.follower.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.rps = Units.rotationsToRadians(this.leader.getVelocity().getValueAsDouble());
    }

    public void setVelocity(double rps) {
        this.leader.setControl(this.velocityRequest.withVelocity(rps));
    }
}