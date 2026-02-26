package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;
import frc.robot.PoseEstimator8736;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    private Transform3d robotToTurret;
    private PoseEstimator8736 poseEstimator;

    public Turret(TurretIO io, Transform3d robotToTurret, PoseEstimator8736 poseEstimator) {
        this.io = io;
        this.robotToTurret = robotToTurret;
        this.poseEstimator = poseEstimator;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        // robot pose on the field
        Pose2d robotPose = poseEstimator.getEstimatedPose();
        // turret pose on the field (robot pose + transform from robot to turret)
        Pose2d turretPose = robotPose.plus(new Transform2d(
            robotToTurret.getTranslation().toTranslation2d(),
            robotToTurret.getRotation().toRotation2d())
        );

        // turret translation
        Translation2d turretToHub = CONSTANTS.Hub.CENTER_BLUE_POSE.toPose2d().getTranslation().minus(turretPose.getTranslation());

        // The angle from the turret to the hub, relative to the field
        Rotation2d turretToHubAngle = new Rotation2d(turretToHub.getX(), turretToHub.getY());
        // The angle that the turret should rotate to, relative to the drivetrain
        Rotation2d desiredAngle = turretToHubAngle.minus(robotPose.getRotation());

        Logger.recordOutput("Simulation/Hub", CONSTANTS.Hub.CENTER_BLUE_POSE);

        Logger.recordOutput("Turret/pose", new Pose2d(
            turretPose.getTranslation(),
            turretPose.getRotation().plus(Rotation2d.fromRadians(inputs.positionRadians)))
        );
        Logger.recordOutput("Turret/desiredPose", new Pose2d(
            turretPose.getTranslation(),
            turretToHubAngle
        ));

        io.setPosition(desiredAngle);
    }

    public void zero() {
        io.zero();
    }
}
