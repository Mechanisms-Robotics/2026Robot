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

    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingTreeMap<Double, Rotation2d> scoreHoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap scoreRPMMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingTreeMap<Double, Rotation2d> shuttleHoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap shuttleRPMMap = new InterpolatingDoubleTreeMap();
    
    /**
     * @param shooterPoseSupplier position of the shooter mechanism field relative, used for distance calculation.
     * @param shooterVelocity velocity of the robot field relative, used for shoot-on-the-move.
     */
    public ShotCalculator(Supplier<Pose3d> shooterPoseSupplier, Supplier<ChassisSpeeds> shooterVelocity) {
        this.shooterPoseSupplier = shooterPoseSupplier;
        this.shooterVelocity = shooterVelocity;

        this.scoreHoodAngleMap.put(1.898, Rotation2d.fromDegrees(24.29));
        this.scoreHoodAngleMap.put(2.284, Rotation2d.fromDegrees(28.67));
        this.scoreHoodAngleMap.put(2.713, Rotation2d.fromDegrees(29.82));
        this.scoreHoodAngleMap.put(3.260, Rotation2d.fromDegrees(31.93));
        this.scoreHoodAngleMap.put(3.618, Rotation2d.fromDegrees(33.96));
        this.scoreHoodAngleMap.put(4.015, Rotation2d.fromDegrees(36.76));
        this.scoreHoodAngleMap.put(4.451, Rotation2d.fromDegrees(34.74));
        this.scoreHoodAngleMap.put(5.332, Rotation2d.fromDegrees(36.06));



        this.scoreRPMMap.put(1.898, 3300.0);
        this.scoreRPMMap.put(2.284, 3400.0);
        this.scoreRPMMap.put(2.713, 3500.0);
        this.scoreRPMMap.put(3.260, 3700.0);
        this.scoreRPMMap.put(3.618, 3900.0);
        this.scoreRPMMap.put(4.015, 4100.0);
        this.scoreRPMMap.put(4.451, 4200.0);
        this.scoreRPMMap.put(5.332, 4500.0);

        this.shuttleHoodAngleMap.put(1.05, Rotation2d.fromDegrees(35.9));
        this.shuttleHoodAngleMap.put(2.22, Rotation2d.fromDegrees(41.0));
        this.shuttleHoodAngleMap.put(3.79, Rotation2d.fromDegrees(48.54));
        this.shuttleHoodAngleMap.put(5.75, Rotation2d.fromDegrees(47.75));
        this.shuttleHoodAngleMap.put(10.11, Rotation2d.fromDegrees(45));

        this.shuttleRPMMap.put(1.05, 3000.0);
        this.shuttleRPMMap.put(2.22, 3200.0);
        this.shuttleRPMMap.put(3.79, 3800.0);
        this.shuttleRPMMap.put(5.75, 4600.0);
        this.shuttleRPMMap.put(10.11, 7000.0);

        this.timeOfFlightMap.put(1.98, 0.86);
        this.timeOfFlightMap.put(2.58, 0.86);
        this.timeOfFlightMap.put(2.99, 1.0);
        this.timeOfFlightMap.put(3.65, 0.85);
        this.timeOfFlightMap.put(4.57, 1.09);
        this.timeOfFlightMap.put(5.56, 1.03);
    }

    public record ShotData(
        Rotation2d shooterYaw,
        Rotation2d hoodAngle,
        double rpm
    ) {}

    /**
     * Calculates desired shooter values based on the target. 
     * @param target target position
     * @param shuttle whether or not this is shuttling. if true, then the shuttling interpolation maps will be used, otherwise the scoring maps will be used
     * @return shot data
     */
    public ShotData calculateShot(Pose2d target, boolean shuttle) {
        Pose2d shooterPose = this.shooterPoseSupplier.get().toPose2d();
        ChassisSpeeds shooterVelocity = this.shooterVelocity.get();
        double targetDistance = target.relativeTo(shooterPose).getTranslation().getNorm();

        
        // the new, lookahead target based on the robot's velocity and time of flight.
        double timeOfFlight = timeOfFlightMap.get(targetDistance);
        Pose2d lookAheadPose = shooterPose;
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
            lookAheadPose =
                new Pose2d(
                    shooterPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    shooterPose.getRotation());
            lookAheadTargetDistance = target.getTranslation().getDistance(lookAheadPose.getTranslation());   
        }

        Translation2d shooterToTarget = target.getTranslation().minus(lookAheadPose.getTranslation());

        Rotation2d desiredHoodAngle = shuttle ? (this.shuttleHoodAngleMap.get(lookAheadTargetDistance)) : (this.scoreHoodAngleMap.get(lookAheadTargetDistance));
        double desiredRPM = shuttle ? this.shuttleRPMMap.get(lookAheadTargetDistance) : this.scoreRPMMap.get(lookAheadTargetDistance);

        Rotation2d desiredYaw = shooterToTarget.getAngle();
        
        Logger.recordOutput("ShotCalculator/targetDistance", targetDistance);
        Logger.recordOutput("ShotCalculator/targetPose", target);
        Logger.recordOutput("ShotCalculator/lookAheadTargetDistance", lookAheadTargetDistance);
        Logger.recordOutput("ShotCalculator/lookAheadTarget", lookAheadPose);
        Logger.recordOutput("ShotCalculator/DesiredHoodDegrees", desiredHoodAngle.getDegrees());
        Logger.recordOutput("ShotCalculator/DesiredRPM", desiredRPM);
        Logger.recordOutput("ShotCalculator/DesiredYawDegrees", desiredYaw.getDegrees());
        
        return new ShotData(desiredYaw, desiredHoodAngle, desiredRPM);
    }
}
