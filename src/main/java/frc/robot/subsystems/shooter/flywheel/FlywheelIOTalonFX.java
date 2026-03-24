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

    private double desiredRPM = 0.0;

    public FlywheelIOTalonFX() {
        PhoenixUtil.tryUntilOk(5, () -> this.leader.getConfigurator().apply(FlywheelConstants.LEADER_CONFIG));
        this.follower.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.rpm = this.leader.getVelocity().getValueAsDouble() * 60.0;
        inputs.desiredRpm = this.desiredRPM;
    }

    @Override
    public void setVelocity(double rpm) {
        this.desiredRPM = rpm; 
        this.leader.setControl(this.velocityRequest.withVelocity(desiredRPM / 60.0));
    }

    @Override
    public void setVoltage(double voltage) {
        this.leader.setVoltage(voltage);
        this.desiredRPM = 0.0;
    }

    @Override
    public double getDesiredVelocity() {
        return this.desiredRPM;
    }
}