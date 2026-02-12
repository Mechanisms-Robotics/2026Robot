package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;
import frc.robot.PoseEstimator8736;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    private Transform2d robotToTurret;
    private PoseEstimator8736 poseEstimator;

    public Turret(TurretIO io, Transform2d robotToTurret, PoseEstimator8736 poseEstimator) {
        this.io = io;
        this.robotToTurret = robotToTurret;
        this.poseEstimator = poseEstimator;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        // calculate desired turret angle and feed it here 
        Pose2d robotPose = poseEstimator.getEstimatedPose();
        Pose2d turretPose = robotPose.plus(robotToTurret);
        Transform2d turretToHub = CONSTANTS.Hub.CENTER_BLUE_POSE.minus(turretPose);

        double turretToHubAngleAbsolute = Math.atan2(turretToHub.getY(), turretToHub.getX());
        double desiredAngle = turretToHubAngleAbsolute - robotPose.getRotation().getRadians();


        Logger.recordOutput("Turret/pose", new Pose2d(turretPose.getTranslation(), Rotation2d.fromRadians(inputs.positionRadians)));
        Logger.recordOutput("Turret/desiredAngle", desiredAngle);

        io.setPosition(desiredAngle);
    }

    public void zero() {
        io.zero();
    }
}
