package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CONSTANTS.TurretConstants;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;

public class SuperStructure extends SubsystemBase {
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private final Feeder feeder;
    private final PoseEstimator8736 poseEstimator;
    private final ShotCalculator shotCalculator;

    private final Command aimHubCommand;
    private final Command aimShuttleCommand;
    private final Command shootCommand;

    public SuperStructure(Flywheel flywheel, Turret turret, Hood hood, Feeder feeder, PoseEstimator8736 poseEstimator, Trigger shootButton, Trigger intakeButton) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.hood = hood;
        this.feeder = feeder;
        this.poseEstimator = poseEstimator;
        this.shotCalculator = new ShotCalculator(
            () -> new Pose3d(
                this.poseEstimator.getEstimatedPose().transformBy(
                    new Transform2d(
                        TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
                        TurretConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()
                    )
                )
            )
        );
        
        this.aimHubCommand = ShootCommands.aimHubCommand(
            this.hood,
            this.flywheel,
            this.turret,
            this.shotCalculator,
            this.poseEstimator
        );

        this.aimShuttleCommand = ShootCommands.aimShuttleCommand(
            this.hood,
            this.flywheel,
            this.turret,
            this.shotCalculator,
            this.poseEstimator
        );

        this.shootCommand = new ShootCommands.Shoot(this.feeder);

        shootButton.and(this::isAimed).whileTrue(this.shootCommand);

        shootButton.whileTrue(
            Commands.either(
                this.aimHubCommand,
                this.aimShuttleCommand,
                () -> FieldUtil.inAllianceZone(this.poseEstimator.getEstimatedPose().getX())
            )
        );
    }

    @Override
    public void periodic() {
        Pose2d robotPose = this.poseEstimator.getEstimatedPose();

        Logger.recordOutput("SuperStructure/ShooterPose3d", 
            new Pose3d(
                new Translation3d(robotPose.getTranslation()).plus(TurretConstants.ROBOT_TO_TURRET.getTranslation()),
                new Rotation3d(
                    0.0,
                    this.hood.getAngle().getRadians() - Math.PI/2.0,
                    this.turret.getAngle().getRadians() + robotPose.getRotation().getRadians()
                )
            ).rotateBy(TurretConstants.ROBOT_TO_TURRET.getRotation())
        );

        Logger.recordOutput("SuperStructure/AimingHub", this.aimHubCommand.isScheduled());
        Logger.recordOutput("SuperStructure/AimingShuttle", this.aimShuttleCommand.isScheduled());
        Logger.recordOutput("SuperStructure/aimed", this.isAimed());
        Logger.recordOutput("SuperStructure/shooting", this.shootCommand.isScheduled());
    }

    public boolean isAimed() {
        return ShootCommands.Aim.anyAimed();
    }
}
