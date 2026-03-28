package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.CONSTANTS.IntakeConstants;

public class RollersIOSparkMax implements RollersIO {
    private final SparkMax motor = new SparkMax(IntakeConstants.ROLLERS_CAN_ID, MotorType.kBrushless);

    public RollersIOSparkMax() {
        this.motor.configure(IntakeConstants.CONFIG_ROLLERS, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        inputs.currentAmps = this.motor.getOutputCurrent();
        inputs.isConnected = this.motor.getLastError() == REVLibError.kOk;
        inputs.tempFahrenheit = this.motor.getMotorTemperature() * 9/5 + 32;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        this.motor.set(dutyCycle);
    }
}
