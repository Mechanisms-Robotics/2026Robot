package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class ShotCalculator {
    private final PoseEstimator8736 poseEstimator;
    private final DoubleSupplier turretPositionSupplier;
    private final DoubleSupplier hoodAngleSupplier;
    private final DoubleSupplier rpsSupplier;
    
    public ShotCalculator(
        PoseEstimator8736 poseEstimator,
        DoubleSupplier turretPosition,
        DoubleSupplier hoodAngle,
        DoubleSupplier rps
    ) {
        this.poseEstimator = poseEstimator;
        this.turretPositionSupplier = turretPosition;
        this.hoodAngleSupplier = hoodAngle;
        this.rpsSupplier = rps;
    }

    public record ShotData(
        boolean aimed,
        double turretAngle,
        double hoodAngle,
        double rps
    ) {}

    public ShotData getShot(Pose3d target) {
        return new ShotData(false, 0.0, 0.0, 0.0);
    }

    public ShotData getShot(Pose2d target) {
        return getShot(new Pose3d(target));
    }
}
