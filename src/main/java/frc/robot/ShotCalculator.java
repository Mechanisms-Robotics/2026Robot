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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCalculator {
    private final Supplier<Pose3d> shooterPoseSupplier;
    private final Supplier<ChassisSpeeds> shooterVelocity;

    private final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    /**
     * @param shooterPoseSupplier position of the shooter mechanism field relative, used for distance calculation.
     * @param shooterVelocity velocity of the robot field relative, used for shoot-on-the-move.
     */
    public ShotCalculator(Supplier<Pose3d> shooterPoseSupplier, Supplier<ChassisSpeeds> shooterVelocity) {
        this.shooterPoseSupplier = shooterPoseSupplier;
        this.shooterVelocity = shooterVelocity;

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


        timeOfFlightMap.put(1.98, 0.86);
        timeOfFlightMap.put(2.58, 0.86);
        timeOfFlightMap.put(2.99, 1.0);
        timeOfFlightMap.put(3.65, 0.85);
        timeOfFlightMap.put(4.57, 1.09);
        timeOfFlightMap.put(5.56, 1.03);
    }

    public record ShotData(
        Rotation2d shooterYaw,
        Rotation2d hoodAngle,
        double rpm
    ) {}

    public ShotData calculateShot(Pose2d target) {
        Pose2d shooterPose = this.shooterPoseSupplier.get().toPose2d();
        ChassisSpeeds shooterVelocity = this.shooterVelocity.get();
        double targetDistance = target.relativeTo(shooterPose).getTranslation().getNorm();

        
        // the new, lookahead target based on the robot's velocity and time of flight.
        double timeOfFlight = timeOfFlightMap.get(targetDistance);
        Pose2d lookAheadTarget = shooterPose;
        double lookAheadTargetDistance = targetDistance;
        
        /* Account for the velocity of the robot by calculating a new target to point at */
        for (int i = 0; i < 20; i++) {
            timeOfFlight = 
                timeOfFlightMap.get(lookAheadTargetDistance);
            /* Calculate the new target to shoot at based on the velocity of the robot (and therefore shooter)
               and the time of flight for the 'most up to date time of flight'. */
            // The x and y offsets based on the 
            double offsetX = shooterVelocity.vxMetersPerSecond * timeOfFlight;
            double offsetY = shooterVelocity.vyMetersPerSecond * timeOfFlight;
            lookAheadTarget =
                new Pose2d(
                    shooterPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    shooterPose.getRotation());
            lookAheadTargetDistance = target.getTranslation().getDistance(lookAheadTarget.getTranslation());   
        }

        Translation2d shooterToTarget = lookAheadTarget.minus(shooterPose).getTranslation();
        Rotation2d desiredYaw = shooterToTarget.getAngle();
        
        Rotation2d desiredHoodAngle = this.hoodAngleMap.get(lookAheadTargetDistance);
        double desiredRPM = this.rpmMap.get(lookAheadTargetDistance);

        Logger.recordOutput("ShotCalculator/targetDistance", targetDistance);
        Logger.recordOutput("ShotCalculator/lookAheadTargetDistance", lookAheadTargetDistance);
        
        return new ShotData(desiredYaw, desiredHoodAngle, desiredRPM);
    }
}
