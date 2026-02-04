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
}
