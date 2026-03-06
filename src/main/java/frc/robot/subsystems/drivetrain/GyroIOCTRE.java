package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.DriveConstants;
import java.util.Queue;

public class GyroIOCTRE implements GyroIO {

    private final Pigeon2 gyro = new Pigeon2(CONSTANTS.GYRO_CAN_ID);

    private final Queue<Double> yawTimestampQueue;
    private final Queue<Double> yawPositionQueue;

    public GyroIOCTRE() {
        // Configure update frequencies to mirror the Redux setup as closely as possible.
        // Phoenix 6 uses per-signal update frequencies (Hz).
        gyro.getYaw().setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
        gyro.getAngularVelocityZWorld().setUpdateFrequency(
            DriveConstants.GRYO_CAN_FRAME_FREQUENCY
        );

        // Match prior behavior
        gyro.setYaw(0.0); // degrees
        gyro.clearStickyFaults();

        // Register the gyro signals
        yawTimestampQueue =
            PhoenixOdometryThread.getInstance().makeTimestampQueue();

        // Phoenix 6 yaw is degrees; convert to rotations to keep the rest of the pipeline identical.
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(
            () -> gyro.getYaw().getValueAsDouble() / 360.0
        );
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // "connected" heuristic: if the yaw StatusSignal status is OK, device is present/responding.
        StatusCode yawStatus = gyro.getYaw().getStatus();
        inputs.connected = yawStatus.isOK();

        // Yaw is degrees -> Rotation2d
        inputs.yawPosition = Rotation2d.fromDegrees(
            gyro.getYaw().getValueAsDouble()
        );

        // Z world angular velocity is deg/sec -> rad/sec
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
            gyro.getAngularVelocityZWorld().getValueAsDouble()
        );

        inputs.odometryYawTimestamps = yawTimestampQueue
            .stream()
            .mapToDouble((Double value) -> value)
            .toArray();

        inputs.odometryYawPositions = yawPositionQueue
            .stream()
            .map(Rotation2d::fromRotations)
            .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void zeroGyro() {
        gyro.setYaw(0.0); // degrees
    }
}
