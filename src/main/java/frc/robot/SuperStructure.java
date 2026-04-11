package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CONSTANTS.TurretConstants;
import frc.robot.ShotCalculator.ShotData;
import frc.robot.CONSTANTS.ManualModeConstants;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

public class SuperStructure extends SubsystemBase {
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private final Feeder feeder;
    private final Intake intake;

    private final PoseEstimator8736 poseEstimator;
    private final ShotCalculator shotCalculator;

    private final ShootCommands.Aim aimCommand;
    private final ShootCommands.Shoot shootCommand;
    private final ShootCommands.ManualShoot manualShootCommand;
    //private final ShootCommands.WobbleSpindexer wobbleSpindexerCommand;
    private final Command intakeCommand;
    private final Command stowCommand;

    private final Trigger shootButton;
    private final Trigger intakeButton;
    private final Trigger manualButton;

    private boolean manualMode = false;  // for Albany we start in auto aim

    public SuperStructure(
        Flywheel flywheel,
        Turret turret,
        Hood hood,
        Feeder feeder,
        Intake intake,
        PoseEstimator8736 poseEstimator,
        ShotCalculator shotCalculator,
        Trigger shootButton,
        Trigger intakeButton,
        Trigger manualButton,
        Trigger stowButton
    ) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.hood = hood;
        this.feeder = feeder;
        this.intake = intake;

        this.poseEstimator = poseEstimator;
        this.shotCalculator = shotCalculator;

        this.shootButton = shootButton;
        this.intakeButton = intakeButton;
        this.manualButton = manualButton;
        
        this.aimCommand = new ShootCommands.Aim(
            this.flywheel, 
            this.turret, 
            this.shotCalculator, 
            this.poseEstimator
        );

        this.shootCommand = new ShootCommands.Shoot(
            this.feeder,
            this.hood,
            this.aimCommand::getShot,
            this::isAimed
        );

        this.manualShootCommand = new ShootCommands.ManualShoot(
            this.flywheel,
            this.feeder,
            ManualModeConstants.FLYWHEEL_RPM
        );

        // this.wobbleSpindexerCommand = new ShootCommands.WobbleSpindexer(
        //     this.feeder
        // );

        this.intakeCommand = IntakeCommands.intake(this.intake);

        this.stowCommand = IntakeCommands.stow(this.intake);

        shootButton.and(() -> !this.manualMode).whileTrue(this.shootCommand);

        // aimCommand handles switching between shooting and shuttling
        new Trigger(() -> !this.manualMode).whileTrue(
            this.aimCommand
        );

        /* Start aim command when the shoot button is pressed and in automatic mode
           This is here because aimCommand doesn't start in teleop (and similar situations)
           This should only be in affect once at the start of teleop */
        shootButton.and(() -> !this.manualMode).and(() -> !this.aimCommand.isScheduled()).onTrue(this.aimCommand);
        new Trigger(() -> this.manualMode).and(() -> this.aimCommand.isScheduled()).onTrue(
            new InstantCommand(() -> this.aimCommand.cancel())
        );

        shootButton.and(() -> this.manualMode).whileTrue(this.manualShootCommand);

        manualButton.onTrue(new InstantCommand(() -> {
            this.manualMode = !this.manualMode;

            this.poseEstimator.setVisionEnabled(!this.manualMode); // disable vision in manual mode to prevent pose jumps

            if (this.manualMode) {
                this.hood.stow();
                this.turret.setAngle(Rotation2d.fromDegrees(TurretConstants.START_DEGREES));
            }
                
        }, this.hood, this.turret));

        intakeButton.whileTrue(this.intakeCommand);//.alongWith(this.wobbleSpindexerCommand));
        //shootButton.onTrue(new InstantCommand(() -> this.wobbleSpindexerCommand.cancel()));
        stowButton.onTrue(this.stowCommand);
    }


    @Override
    public void periodic() {
        // Used to animate mechanisms
        // In advantage scope, drag this array onto the robot pose to animate mechanisms
        Logger.recordOutput("SuperStructure/Components", 
            new Pose3d[] {
                // Turret
                new Pose3d(0, 0, 0, new Rotation3d(
                    0.0,
                    0.0,
                    this.turret.getAngle().getRadians()
                )),
                // Hood
                new Pose3d(
                    this.turret.getAngle().getCos() * TurretConstants.CENTER_TO_HOOD_PIVOT_METERS,
                    this.turret.getAngle().getSin() * TurretConstants.CENTER_TO_HOOD_PIVOT_METERS,
                    0.386123,
                    new Rotation3d(
                        0.0,
                        this.hood.getAngle().getRadians(),
                        this.turret.getAngle().getRadians()
                )),
                // Intake
                new Pose3d(0.298, 0, 0.16, new Rotation3d(
                    0.0,
                    -this.intake.getAngle().getRadians(),
                    0.0
                ))
            }
        );

        Logger.recordOutput("SuperStructure/Aiming", this.aimCommand.isScheduled());
        Logger.recordOutput("SuperStructure/Aimed", this.isAimed());
        Logger.recordOutput("SuperStructure/WithinSoftLimits", this.isWithinSoftLimits());
        Logger.recordOutput("SuperStructure/Shooting", this.shootCommand.isScheduled());
        Logger.recordOutput("SuperStructure/Intaking", this.intakeCommand.isScheduled());
        Logger.recordOutput("SuperStructure/Stowing", this.stowCommand.isScheduled());
        //Logger.recordOutput("SuperStructure/WobbleSpindexer", this.wobbleSpindexerCommand.isScheduled());
        Logger.recordOutput("SuperStructure/ManualMode", this.manualMode);
        Logger.recordOutput("SuperStructure/Buttons/Shoot", this.shootButton.getAsBoolean());
        Logger.recordOutput("SuperStructure/Buttons/Intake", this.intakeButton.getAsBoolean());
        Logger.recordOutput("SuperStructure/Buttons/ManualToggle", this.manualButton.getAsBoolean());
    }

    public boolean isAimed() {
        ShotData shotData = this.aimCommand.getShot();
        Rotation2d shooterYaw =
            this.poseEstimator
                .getEstimatedPose()
                .getRotation()
                .plus(this.turret.getAngle());
        Rotation2d desiredShooterYaw = shotData.shooterYaw();

        return Math.abs(shooterYaw.relativeTo(desiredShooterYaw).getDegrees()) < 10.0
            && Math.abs(this.flywheel.getRPM() - shotData.rpm()) < 500.0;
    }

    /**
     * Returns true if the turret is within the soft limits, aka the turret is in a good position.
     */
    public boolean isWithinSoftLimits() {
        Rotation2d desiredShooterYaw = this.aimCommand.getShot().shooterYaw();

        return (desiredShooterYaw.getDegrees() >= TurretConstants.MIN_DEGREES) && desiredShooterYaw.getDegrees() <= TurretConstants.MAX_DEGREES;
    }
}
