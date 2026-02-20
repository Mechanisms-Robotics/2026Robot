package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CONSTANTS;

public class ClimberIOTalonFX implements ClimberIO {
    private final ClimberIOInputs inputs = new ClimberIOInputs();

    private final TalonFX climberMotor;
    private final CANcoder encoder;

    public ClimberIOTalonFX() {
        climberMotor = new TalonFX(CONSTANTS.CLIMBER_MOTOR_ID);
        encoder = new CANcoder(CONSTANTS.CLIMBER_ENCODER_ID);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberConnected = true;

        // try {
        //     inputs.climberOutputPercent = this.climberMotor.getMotorOutputPercent();
        // } catch (Exception e) {
        //     inputs.climberOutputPercent = Double.NaN;
        // }
        
        // double busVolt = Double.NaN;
        // try {
        //     busVolt = this.climberMotor.getBusVoltage();
        // } catch (Exception e) {
        //     busVolt = Double.NaN;
        // }

        // if (!Double.isNaN(busVolt) && !Double.isNaN(inputs.climberOutputPercent)) {
        //     inputs.climberAppliedVolts = inputs.climberOutputPercent * busVolt;
        // } else if (!Double.isNaN(inputs.climberOutputPercent)) {
        //     inputs.climberAppliedVolts = inputs.climberOutputPercent * 12.0;
        // } else {
        //     inputs.climberAppliedVolts = Double.NaN;
        // }

        // try {
        //     double velocityRps = this.encoder.getVelocity() / CONSTANTS.CLIMBER_GEAR_RATIO;
        //     inputs.climberVelocityRotationsPerSec = velocityRps;
        // } catch (Exception e) {
        //     inputs.climberVelocityRotationsPerSec = Double.NaN;
        // }
    }

    @Override
    public void setMotorOpenLoop(double output) {
        this.climberMotor.setVoltage(output);
    }

    @Override
    public void setMotorVelocity(double speedRotPerSec) {}

    @Override
    public void stopMotor() {
        climberMotor.stopMotor();
    }

}
