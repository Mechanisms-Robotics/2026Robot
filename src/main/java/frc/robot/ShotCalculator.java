package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.Timer;

public class ShotCalculator {
    private final Supplier<Pose3d> shooterPoseSupplier;
    private final Supplier<Pose2d> robotOdometrySupplier;

    private final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private double dt;
    private Pose2d lastOdometryPose;
    
    /**
     * @param shooterPoseSupplier position of the shooter mechanism field relative, used for distance calculation.
     * @param robotOdometrySupplier velocity of the robot, used for shoot-on-the-move.
     */
    public ShotCalculator(Supplier<Pose3d> shooterPoseSupplier, Supplier<Pose2d> robotOdometrySupplier) {
        this.shooterPoseSupplier = shooterPoseSupplier;
        this.robotOdometrySupplier = robotOdometrySupplier;

        this.hoodAngleMap.put(1.0, Rotation2d.fromDegrees(22.0));
        this.hoodAngleMap.put(4.0, Rotation2d.fromDegrees(22.0));

        this.rpmMap.put(1.47, 2700.0);
        this.rpmMap.put(2.12, 2900.0);
        this.rpmMap.put(2.99, 3300.0);
        this.rpmMap.put(3.72, 3500.0);
        this.rpmMap.put(4.49, 3700.0);

        // just examples for now. this needs to be tuned
        timeOfFlightMap.put(1.0, 0.5);
        timeOfFlightMap.put(2.0, 1.0);

        lastOdometryPose = robotOdometrySupplier.get();
    }

    public record ShotData(
        Rotation2d shooterYaw,
        Rotation2d hoodAngle,
        double rpm
    ) {}

    public ShotData calculateShot(Pose3d target) {
        // stubbed for now. velocity of robot, in m/s
        double robotVelocityX = 0;
        double robotVelocityY = 0;

        Pose3d shooterPose = this.shooterPoseSupplier.get();
        double targetDistance = target.relativeTo(shooterPose).getTranslation().getNorm();

        double timeOfFlight = timeOfFlightMap.get(targetDistance);

        // the new, lookahead target based on the robot's velocity and time of flight.
        Pose3d lookAheadTarget = new Pose3d(
            new Translation3d(
                target.getX() - (robotVelocityX * timeOfFlight),
                target.getY() - (robotVelocityY * timeOfFlight),
                target.getZ()
            ),
            Rotation3d.kZero
        );

        double lookAheadTargetDistance = lookAheadTarget.relativeTo(shooterPose).getTranslation().getNorm();
        
        Rotation2d desiredHoodAngle = this.hoodAngleMap.get(lookAheadTargetDistance);
        double desiredRPM = this.rpmMap.get(lookAheadTargetDistance);

        Translation2d shooterToTarget = lookAheadTarget.getTranslation().minus(shooterPose.getTranslation()).toTranslation2d();
        Rotation2d desiredYaw = shooterToTarget.getAngle();

        Logger.recordOutput("ShotCalculator/targetDistance", targetDistance);
        Logger.recordOutput("ShotCalculator/lookAheadTargetDistance", lookAheadTargetDistance);
        
        return new ShotData(desiredYaw, desiredHoodAngle, desiredRPM);
    }

    public ShotData calculateShot(Pose2d target) {
        return calculateShot(new Pose3d(target));
    }
}
