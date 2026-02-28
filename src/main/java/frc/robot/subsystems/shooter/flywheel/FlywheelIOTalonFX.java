package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.CONSTANTS.FlywheelConstants;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final TalonFX leader = new TalonFX(FlywheelConstants.LEADER_ID);
    private final TalonFX follower = new TalonFX(FlywheelConstants.FOLLOWER_ID);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    public FlywheelIOTalonFX() {
        PhoenixUtil.tryUntilOk(5, () -> this.leader.getConfigurator().apply(FlywheelConstants.LEADER_CONFIG));
        this.follower.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.rps = this.leader.getVelocity().getValueAsDouble();
    }

    public void setVelocity(double rps) {
        this.leader.setControl(this.velocityRequest.withVelocity(rps));
    }

    public void stopPower() {
        this.leader.setVoltage(0.0);
    }
}