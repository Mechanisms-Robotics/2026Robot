package frc.robot.subsystems.shooter.turret;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Timeouts.TurretConstants;

/**
 * The IO layer of the turret for the competition robot.
 * 
 * Uses a neo to turn the turret.
 * Uses the incremental encoder on the neo to track velocity. The incremental encoders
 * are a good way to keep track of velocity. DutyCycleEncoder doesn't keep track of velocity.
 * 
 * Uses two REV through bore encoders to keep track of the absolute position of the turret.
 * One encoder is not enough to keep track of absolute position (between boots) because the
 * encoder does not keep track of wrap around and the gear on the encoder is much smaller
 * than the turret gear.
 * 
 * Two encoders allows leverage of Chinese Remainder Theorem. Adding a second encoder is
 * analogous to adding another digit to a number. One decimal digit can represent 10 states (0-9).
 * Two decimal digits can represent 100 states (0-100). The total states that can be represented
 * by the gears is close to digits. The total states for digits is 
 * digit1_states * digit2_states = total_states. For decimal this is 10 * 10 = 100. Binary: 2 * 2 = 4. 
 * For the turret it is (encoder1_ratio * encoder2_ratio) / (turret_teeth * GCF_of_encoders) =
 * trackable rotations. encoder1_teeth and encoder2_teeth are the gear ratios of the encoders not
 * including the turret gear. The GCF (https://en.wikipedia.org/wiki/Greatest_common_divisor) of the
 * encoders accounts for duplicate states. For example, if the GCF = encoder1_ratio = encoder2_ratio,
 * i.e. the gear ratios of the encoder are the same, then ratio/turret_teeth = trackable_rotations.
 * This is only as helpful as having a single encoder. If the ratios are coprime (GCF is 1), this
 * equation becomes (encoder1_ratio * encoder2_ratio) / turret_teeth. The darn vendors don't supply
 * odd number teeth so we are limited to having a GCF of 2.
 * 
 * Two REV through bore encoders (as of 2/19, 2 v1s):
 * https://www.revrobotics.com/rev-11-1271
 * https://www.revrobotics.com/rev-11-3174
 * 
 * Neo and Spark Max
 * https://www.revrobotics.com/rev-21-1650
 * https://www.revrobotics.com/rev-11-2158
 */
public class TurretIOSparkMax extends SubsystemBase implements TurretIO {
    // Hardware
    private final SparkMax motor = new SparkMax(20, MotorType.kBrushless);
    private final RelativeEncoder motorEncoder = motor.getEncoder();
    private final DutyCycleEncoder encoder1 = new DutyCycleEncoder(0);
    private final DutyCycleEncoder encoder2 = new DutyCycleEncoder(1);
    
    // changed this to rotation2d because easier to work with
    private Rotation2d desiredPosition = Rotation2d.kZero;

    public TurretIOSparkMax() {
        this.motor.configure(TurretConstants.CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.velocityRadiansPerSec = Units.rotationsToRadians(this.getVelocity());

        Optional<Rotation2d> position = this.getPosition();
        if (position.isPresent()) {
            inputs.positionRadians = position.get().getRadians();
            
            // Wrap around logic.

            // Shortest possible rotation for the turret to move, which if done may break the wire chain
            Rotation2d relative = desiredPosition.relativeTo(Rotation2d.fromRadians(inputs.positionRadians));
            // Error in the PID sence
            double error = relative.getRadians();
            // The desired position including the number of times the turret has made a revolution
            double target = inputs.positionRadians + relative.getRadians();

            if (target > TurretConstants.FORWARD_LIMIT || target < TurretConstants.REVERSE_LIMIT) {
                // Changes direction by reversing the magnitude.
                error -= Math.signum(error) * (Math.PI * 2);
            }

            double appliedVoltage = error * TurretConstants.kP
                                   - this.getVelocity() * TurretConstants.kD;

            this.motor.setVoltage(appliedVoltage);
            Logger.recordOutput("Turret/appliedVoltage", appliedVoltage);
        } else {
            inputs.positionRadians = 0.0;
            this.motor.setVoltage(0.0);
        }
    }

    /**
     * Sets a desired position for the turret, in radians.
     */
    @Override
    public void setAngle(Rotation2d position) {
        this.desiredPosition = position;
    }

    /**
     * Get velocity of the turret
     * 
     * @return velocity of the turret in RPM
     */
    public double getVelocity() {
        return this.motorEncoder.getVelocity() * TurretConstants.MOTOR_GEAR_RATIO;
    }

    /**
     * Get the absolute position of the turret as a Rotation2d.
     * This is calculated from two encoders using Chinese Remainder Theorem.
     * 
     * @return turret position as a Rotation2d
     */
    public Optional<Rotation2d> getPosition() {
        final double tolerance = 0.05;
        // Absolute position of encoders 1 and 2
        final double abs1 = MathUtil.inputModulus(this.encoder1.get(), 0.0, 1.0);
        final double abs2 = MathUtil.inputModulus(this.encoder2.get(), 0.0, 1.0);

        /* Minimum and maximum number of rotations the first encoder could be at WITHOUT wrap around not accounting 
           for the rotation read from the encoder. */
        double minEncoder1Rotations = TurretConstants.RATIO1 * TurretConstants.MIN_POSITION;
        double maxEncoder1Rotations = TurretConstants.RATIO1 * TurretConstants.MAX_POSITION;
        // Convert min and max bounds to integers for looping purposes
        int minPossible = (int) Math.floor(minEncoder1Rotations - abs1) - 1;
        int maxPossible = (int) Math.ceil(maxEncoder1Rotations - abs1) + 1;

        double bestError = Double.POSITIVE_INFINITY;
        double bestPosition = Double.NaN;
        // Keep track of second best error because if there are 2 similarly plausible angles, that no good cuh
        double secondBestError = Double.POSITIVE_INFINITY;

        /* Try every possible turret rotation that matches with encoder 1.
           Calculate the absolutute position encoder 2 would have given each turret rotation.
           Compare with encoder 2's actual position and save the rotation corresponding with the lowest error
           between the corresponding/predicted encoder 2 position and the actual position from the encoder. */
        for (int n = minPossible; n < maxPossible; n++) {
            // Possible position of the turret given encoder1's rotation
            double possiblePosition = (abs1 + n) / TurretConstants.RATIO1;
            // Predicted rotation of encoder2 given the possible position of the turret
            double abs2Pred = MathUtil.inputModulus(TurretConstants.RATIO2 * possiblePosition, 0.0, 1.0);

            double error = Math.abs(abs2Pred - abs2);
            error = error > 0.5 ? 1.0 - error : error;

            if (error < bestError) {
                secondBestError = bestError;
                bestError = error;
                bestPosition = possiblePosition;
            } else if (error < secondBestError) {
                secondBestError = error;
            }
        }

        if (!Double.isFinite(bestPosition)) {
            Logger.recordOutput("Turret/encoder/status", "invalid turret position");
            return Optional.empty();
        }

        if (bestError > tolerance) {
            Logger.recordOutput("Turret/encoder/status", "error " + bestError + " > " + tolerance);
            return Optional.empty();
        }

        if (secondBestError <= tolerance && Math.abs(secondBestError - bestError) < 1e-3) {
            Logger.recordOutput("Turret/encoder/status", "ambiguous position");
            return Optional.empty();
        }

        return Optional.of(
            Rotation2d.fromRotations(bestPosition)
            .plus(Rotation2d.fromRadians(TurretConstants.TURRET_OFFSET)));
    }
}
