package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.CONSTANTS;

public class ClimberIOSparkMax implements ClimberIO {
    private final ClimberIOInputs inputs = new ClimberIOInputs();

    private final SparkMax climberMotor;
    private final RelativeEncoder encoder;

    public ClimberIOSparkMax(int canID) {
        this.climberMotor = new SparkMax(canID, MotorType.kBrushless);
        this.encoder = climberMotor.getEncoder();
    }

    public ClimberIOSparkMax() {
        this.climberMotor = new SparkMax(CONSTANTS.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        this.encoder = climberMotor.getEncoder();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {

        inputs.climberConnected = true;

        try {
            inputs.climberOutputPercent = this.climberMotor.get();
        } catch (Exception e) {
            inputs.climberOutputPercent = Double.NaN;
        }
        
        double busVolt = Double.NaN;
        try {
            busVolt = this.climberMotor.getBusVoltage();
        } catch (Exception e) {
            busVolt = Double.NaN;
        }

        if (!Double.isNaN(busVolt) && !Double.isNaN(inputs.climberOutputPercent)) {
            inputs.climberAppliedVolts = inputs.climberOutputPercent * busVolt;
        } else if (!Double.isNaN(inputs.climberOutputPercent)) {
            inputs.climberAppliedVolts = inputs.climberOutputPercent * 12.0;
        } else {
            inputs.climberAppliedVolts = Double.NaN;
        }
    }

    @Override
    public void setMotorOpenLoop(double output) {
        climberMotor.set(output);
    }

    @Override
    public void setMotorVelocity(double speedRotPerSec) {
        // Convert requested speed to RPS/RPM
        double desiredRPS = speedRotPerSec;
        double desiredRPM = desiredRPS * 60.0;

        // Read actual velocity (encoder returns RPM)
        double actualRPM = Double.NaN;
        try {
            actualRPM = encoder.getVelocity();
        } catch (Exception e) {
            actualRPM = Double.NaN;
        }
        double actualRPS = !Double.isNaN(actualRPM) ? actualRPM / 60.0 : Double.NaN;

        // Simple closed-loop: feedforward (kV * desiredRPS) + proportional (kP * errorRPS)
        // Tweak these gains for your mechanism.
        final double kV = 1.0;   // volts per RPS (feedforward)
        final double kP = 0.2;   // volts per RPS error (proportional)

        double ffVolts = kV * desiredRPS;
        double errorRPS = !Double.isNaN(actualRPS) ? (desiredRPS - actualRPS) : 0.0;
        double pVolts = kP * errorRPS;
        double outputVolts = ffVolts + pVolts;

        // Convert volts -> percent output using bus voltage
        double busVolt = Double.NaN;
        try {
            busVolt = climberMotor.getBusVoltage();
        } catch (Exception e) {
            busVolt = Double.NaN;
        }
        if (Double.isNaN(busVolt) || Math.abs(busVolt) < 1e-6) {
            busVolt = 12.0; // fallback
        }

        double percent = outputVolts / busVolt;
        if (Double.isNaN(percent)) {
            percent = 0.0;
        }
        // clamp
        percent = Math.max(-1.0, Math.min(1.0, percent));

        climberMotor.set(percent);
    }

    @Override
    public void stopMotor() {
        climberMotor.stopMotor();
    }


}
