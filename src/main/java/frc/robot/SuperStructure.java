package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CONSTANTS.TurretConstants;
import frc.robot.ShotCalculator.ShotData;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;

public class SuperStructure extends SubsystemBase {
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private final PoseEstimator8736 poseEstimator;
    private final ShotCalculator shotCalculator;
    private ShotData shotData = new ShotData(null, null, 0);

    private boolean shuttling = false;
    private boolean intaking = false;

    private final Command aimCommand;
    private final Command shootCommand;

    public SuperStructure(Flywheel flywheel, Turret turret, Hood hood, PoseEstimator8736 poseEstimator, Trigger shootButton, Trigger intakeButton) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.hood = hood;
        this.poseEstimator = poseEstimator;
        // TODO: add TurretConstants.ROBOT_TO_TURRET
        this.shotCalculator = new ShotCalculator(
            () -> new Pose3d(this.poseEstimator.getEstimatedPose()));
        
        this.aimCommand = new ShootCommands.Aim(this.hood, this.flywheel, () -> this.shotData);
        this.shootCommand = new ShootCommands.Shoot();

        shootButton.and(() -> this.aimed()).whileTrue(this.shootCommand);
        shootButton.whileTrue(this.aimCommand);

        this.turret.setDefaultCommand(
            new RunCommand(
                () -> this.turret.setAngle(this.shotData.shooterYaw().minus(this.poseEstimator.getEstimatedPose().getRotation()))
            , this.turret)
        );
    }

    @Override
    public void periodic() {
        Pose2d robotPose = this.poseEstimator.getEstimatedPose();
        shotData = FieldUtil.inAllianceZone(robotPose)
            ? this.shotCalculator.calculateShot(FieldUtil.getHub())
            : this.shotCalculator.calculateShot(FieldUtil.getShuttlePose(robotPose.getY()));

        Logger.recordOutput("SuperStructure/ShooterPose3d", 
            new Pose3d(
                new Translation3d(robotPose.getTranslation()).plus(TurretConstants.ROBOT_TO_TURRET.getTranslation()),
                new Rotation3d(
                    0.0,
                    -this.hood.getAngle().getRadians(),
                    this.turret.getAngle().getRadians()
                )
            ).rotateBy(TurretConstants.ROBOT_TO_TURRET.getRotation())
        );

        Logger.recordOutput("SuperStructure/aiming", this.aimCommand.isScheduled());
        Logger.recordOutput("SuperStructure/aimed", this.aimed());
        Logger.recordOutput("SuperStructure/shooting", this.shootCommand.isScheduled());
        Logger.recordOutput("SuperStructure/shuttling", shuttling);
        Logger.recordOutput("SuperStructure/intaking", intaking);
    }

    public boolean aimed() {
        return false;
    }
}
