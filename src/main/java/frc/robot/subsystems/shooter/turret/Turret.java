package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;
import static frc.robot.CONSTANTS.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

        Pose2d goal;

        // Logic to determine where the turret should aim depending on the alliance,
        // if it's in the shuttling zone, and if it's in the top or bottom half of the
        // field. This is mainly just testing
        if (DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get() == Alliance.Red) {
            if (robotPose.getX() < FieldConstants.RED_ALLIANCE_ZONE) {
                if (robotPose.getY() > FieldConstants.CENTER.getY()) {
                    goal = FieldConstants.SHUTTLE_OUTPOST_RED_POSE;
                } else {
                    goal = FieldConstants.SHUTTLE_DEPOT_RED_POSE;
                }
            } else {
                goal = CONSTANTS.Hub.CENTER_RED_POSE.toPose2d();
            }
        } else {
            if (robotPose.getX() > FieldConstants.BLUE_ALLIANCE_ZONE) {
                if (robotPose.getY() > FieldConstants.CENTER.getY()) {
                    goal = FieldConstants.SHUTTLE_DEPOT_BLUE_POSE;
                } else {
                    goal = FieldConstants.SHUTTLE_OUTPOST_BLUE_POSE;
                }
            } else {
                goal = CONSTANTS.Hub.CENTER_BLUE_POSE.toPose2d();
            }
        }


        // turret translation
        Translation2d turretToGoal = goal.getTranslation().minus(turretPose.getTranslation());

        // The angle from the turret to the goal, relative to the field
        Rotation2d turretToGoalAngle = new Rotation2d(turretToGoal.getX(), turretToGoal.getY());
        // The angle that the turret should rotate to, relative to the drivetrain
        Rotation2d desiredAngle = turretToGoalAngle.minus(robotPose.getRotation());

        Logger.recordOutput("Simulation/Goal", goal);

        Logger.recordOutput("Turret/pose",new Pose3d(
            turretPose.getX(), turretPose.getY(), robotToTurret.getZ(),
            new Rotation3d(turretPose.getRotation().plus(Rotation2d.fromRadians(inputs.positionRadians))))
        );
        Logger.recordOutput("Turret/desiredPose", new Pose3d(
            turretPose.getX(), turretPose.getY(), robotToTurret.getZ(),
            new Rotation3d(turretToGoalAngle)
        ));
        Logger.recordOutput("Turret/angle", turretPose.getRotation().getRadians() + inputs.positionRadians);
        Logger.recordOutput("Turret/desiredAngle", turretToGoalAngle.getRadians());

        io.setPosition(desiredAngle);
    }

    public void zero() {
        io.zero();
    }
}
