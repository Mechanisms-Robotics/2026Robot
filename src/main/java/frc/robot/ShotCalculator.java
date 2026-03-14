package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotCalculator {
    private final Supplier<Pose3d> shooterPoseSupplier;

    private final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    
    /**
     * @param shooterPoseSupplier position of the shooter mechanism field relative, used for distance calculation.
     */
    public ShotCalculator(Supplier<Pose3d> shooterPoseSupplier) {
        this.shooterPoseSupplier = shooterPoseSupplier;

        this.hoodAngleMap.put(1.898, Rotation2d.fromDegrees(24.29));
        this.hoodAngleMap.put(2.284, Rotation2d.fromDegrees(28.67));
        this.hoodAngleMap.put(2.713, Rotation2d.fromDegrees(29.82));
        this.hoodAngleMap.put(3.260, Rotation2d.fromDegrees(31.93));
        this.hoodAngleMap.put(3.618, Rotation2d.fromDegrees(33.96));
        this.hoodAngleMap.put(4.015, Rotation2d.fromDegrees(36.76));
        this.hoodAngleMap.put(4.451, Rotation2d.fromDegrees(34.74));
        this.hoodAngleMap.put(5.332, Rotation2d.fromDegrees(36.06));

        this.rpmMap.put(1.898, 3300.0);
        this.rpmMap.put(2.284, 3400.0);
        this.rpmMap.put(2.713, 3500.0);
        this.rpmMap.put(3.260, 3700.0);
        this.rpmMap.put(3.618, 3900.0);
        this.rpmMap.put(4.015, 4100.0);
        this.rpmMap.put(4.451, 4200.0);
        this.rpmMap.put(5.332, 4500.0);
    }

    public record ShotData(
        Rotation2d shooterYaw,
        Rotation2d hoodAngle,
        double rpm
    ) {}

    public ShotData calculateShot(Pose3d target) {
        Pose3d shooterPose = this.shooterPoseSupplier.get();

        double targetDistance = target.relativeTo(shooterPose).getTranslation().getNorm();
        Rotation2d desiredHoodAngle = this.hoodAngleMap.get(targetDistance);
        double desiredRPM = this.rpmMap.get(targetDistance);

        Translation2d shooterToTarget = target.getTranslation().minus(shooterPose.getTranslation()).toTranslation2d();
        Rotation2d desiredYaw = shooterToTarget.getAngle();

        Logger.recordOutput("ShotCalculator/targetDistance", targetDistance);
        
        return new ShotData(desiredYaw, desiredHoodAngle, desiredRPM);
    }

    public ShotData calculateShot(Pose2d target) {
        return calculateShot(new Pose3d(target));
    }
}
