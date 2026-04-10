package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Fahrenheit;

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
        inputs.leaderTempFahrenheit = this.leader.getDeviceTemp().getValue().in(Fahrenheit);
        inputs.followerTempFahrenheit = this.follower.getDeviceTemp().getValue().in(Fahrenheit);
        inputs.leaderCurrentAmps = this.leader.getSupplyCurrent().getValueAsDouble();
        inputs.followerCurrentAmps = this.follower.getSupplyCurrent().getValueAsDouble();
        inputs.leaderConnected = this.leader.isConnected();
        inputs.followerConnected = this.follower.isConnected();
    }

    @Override
    public void setVelocity(double rpm) {
        this.desiredRPM = rpm; 
        this.leader.setControl(this.velocityRequest.withVelocity(desiredRPM / 60.0));
    }

    @Override
    public void setVoltage(double voltage) {
        this.leader.setVoltage(voltage);
        // show desired rpm as 0 when running in open loop
        this.desiredRPM = 0.0;
    }

    @Override
    public double getDesiredVelocity() {
        return this.desiredRPM;
    }
}