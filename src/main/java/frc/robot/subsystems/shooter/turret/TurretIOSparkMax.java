package frc.robot.subsystems.shooter.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CONSTANTS.TurretConstants;

public class TurretIOSparkMax implements TurretIO {
    private final SparkMax motor = new SparkMax(TurretConstants.MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = this.motor.getEncoder();

    public TurretIOSparkMax() {
        this.motor.configure(TurretConstants.CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.zero();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionDegrees = Units.rotationsToDegrees(this.encoder.getPosition());
        inputs.velocityDegreesPerSecond = Units.rotationsToDegrees(this.encoder.getVelocity());
        inputs.setpointDegrees = this.motor.getClosedLoopController().getSetpoint() * 360.0;
        inputs.current = this.motor.getOutputCurrent();
        inputs.connected = this.motor.getLastError() == REVLibError.kOk;
        inputs.tempFahrenheit = this.motor.getMotorTemperature() * 9/5 + 32;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        double currentPosition = this.encoder.getPosition();
        double setpoint = angle.getRotations();

        // for example alt 170 is -190, alt 30 is -330
        double alternateSetpoint = setpoint - Math.signum(setpoint);

        /* If the primary setpoint is past soft limits or the alternate setpoint is a shorter distance
           and within soft limits, use the alternate setpoint. */
        if (!this.withinSoftlimits(setpoint * 360.0)
         || (Math.abs(alternateSetpoint - currentPosition) < Math.abs(setpoint -  currentPosition)
          && this.withinSoftlimits(alternateSetpoint * 360.0))) {
            setpoint = alternateSetpoint;
        }

        this.motor.getClosedLoopController().setSetpoint(setpoint, ControlType.kPosition);
    }

    private boolean withinSoftlimits(double angleDegrees) {
        return angleDegrees >= TurretConstants.MIN_DEGREES
            && angleDegrees <= TurretConstants.MAX_DEGREES;
    }

    @Override
    public void zero() {
        this.encoder.setPosition(Units.degreesToRotations(TurretConstants.START_DEGREES));
    }
}
