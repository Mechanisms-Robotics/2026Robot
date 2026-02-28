package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotCalculator {
    private final Supplier<Pose3d> shooterPoseSupplier;
    private final Supplier<Rotation2d> hoodAngleSupplier;
    private final DoubleSupplier rpsSupplier;

    private final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();
    
    private final double rpsEpsilon = 100.0;
    private final double yawEpsilon = 3.0;
    private final double hoodEpsilon = 3.0;
    
    /**
     * @param shooterPoseSupplier position of the shooter mechanism field relative
     * @param hoodAngle hood rotation2d
     * @param rps rotations per second of the flywheels
     */
    public ShotCalculator(
        Supplier<Pose3d> shooterPoseSupplier,
        Supplier<Rotation2d> hoodAngle,
        DoubleSupplier rps
    ) {
        this.shooterPoseSupplier = shooterPoseSupplier;
        this.hoodAngleSupplier = hoodAngle;
        this.rpsSupplier = rps;

        this.hoodAngleMap.put(0.0, Rotation2d.fromDegrees(10.0));
        this.rpsMap.put(0.0, 10.0);
    }

    public record ShotData(
        boolean aimed,
        Rotation2d shooterYaw,
        Rotation2d hoodAngle,
        double rps
    ) {}

    public ShotData calculateShot(Pose3d target) {
        Pose3d shooterPose = this.shooterPoseSupplier.get();

        double targetDistance = target.relativeTo(shooterPose).getTranslation().getNorm();
        Rotation2d hoodAngle = this.hoodAngleMap.get(targetDistance);
        double rps = this.rpsMap.get(targetDistance);

        Translation2d shooterToTarget = target.getTranslation().minus(shooterPose.getTranslation()).toTranslation2d();
        Rotation2d shooterYaw = shooterToTarget.getAngle();

        boolean aimed =
            Math.abs(rpsSupplier.getAsDouble() - rps) < this.rpsEpsilon
         && Math.abs(shooterYaw.getDegrees() - this.shooterPoseSupplier.get().getRotation().toRotation2d().getDegrees()) < this.yawEpsilon
         && Math.abs(hoodAngle.getDegrees() - this.hoodAngleSupplier.get().getDegrees()) < this.hoodEpsilon;
        
        return new ShotData(aimed, shooterYaw, hoodAngle, rps);
    }

    public ShotData calculateShot(Pose2d target) {
        return calculateShot(new Pose3d(target));
    }
}
