package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CONSTANTS.TurretConstants;
import frc.robot.CONSTANTS.ManualModeConstants;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;

public class SuperStructure extends SubsystemBase {
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private final Feeder feeder;
    private final Intake intake;

    private final PoseEstimator8736 poseEstimator;
    private final ShotCalculator shotCalculator;

    private final Command aimHubCommand;
    private final Command aimShuttleCommand;
    private final Command shootCommand;
    private final Command manualShootCommand;
    private final Command intakeCommand;

    private final Trigger shootButton;
    private final Trigger intakeButton;
    private final Trigger manualButton;

    private boolean manualMode = true;  // for Dalton we start in manual mode

    public SuperStructure(
        Flywheel flywheel,
        Turret turret,
        Hood hood,
        Feeder feeder,
        Intake intake,
        PoseEstimator8736 poseEstimator,
        Trigger shootButton,
        Trigger intakeButton,
        Trigger manualButton
    ) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.hood = hood;
        this.feeder = feeder;
        this.intake = intake;

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

        this.shootButton = shootButton;
        this.intakeButton = intakeButton;
        this.manualButton = manualButton;
        
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

        this.manualShootCommand = new ShootCommands.ManualShoot(
            this.flywheel,
            this.feeder,
            ManualModeConstants.FLYWHEEL_RPM
        );

        this.intakeCommand = IntakeCommands.intake(this.intake);

        shootButton.and(() -> !this.manualMode).and(this::isAimed).whileTrue(this.shootCommand);

        shootButton.and(() -> !this.manualMode).whileTrue(
            Commands.either(
                this.aimHubCommand,
                this.aimShuttleCommand,
                () -> FieldUtil.inAllianceZone(this.poseEstimator.getEstimatedPose().getX())
            )
        );

        shootButton.and(() -> this.manualMode).whileTrue(this.manualShootCommand);

        manualButton.onTrue(new InstantCommand(() -> {
            this.manualMode = !this.manualMode;
            if (this.manualMode)
                this.hood.stow();
        }, this.hood));

        intakeButton.whileTrue(this.intakeCommand);
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
        Logger.recordOutput("SuperStructure/Aimed", this.isAimed());
        Logger.recordOutput("SuperStructure/Shooting", this.shootCommand.isScheduled());
        Logger.recordOutput("SuperStructure/Intaking", this.intakeCommand.isScheduled());
        Logger.recordOutput("SuperStructure/Buttons/Shoot", this.shootButton.getAsBoolean());
        Logger.recordOutput("SuperStructure/Buttons/Intake", this.intakeButton.getAsBoolean());
        Logger.recordOutput("SuperStructure/Buttons/ManualToggle", this.manualButton.getAsBoolean());
    }

    public boolean isAimed() {
        return ShootCommands.Aim.anyAimed();
    }
}
