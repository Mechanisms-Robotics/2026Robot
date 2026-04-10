package frc.robot.commands.autos;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.ShotCalculator;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MaxScoring extends SequentialCommandGroup {
    public MaxScoring(
        Drivetrain drivetrain,
        Hood hood,
        Flywheel flywheel,
        Feeder feeder,
        Turret turret,
        Intake intake,
        ShotCalculator shotCalculator,
        boolean mirror
    ) {
        Optional<Trajectory<SwerveSample>> trenchToNeutralBeach = Choreo.loadTrajectory(
                    "TrenchToNeutralLeftBeach"
                );
        Optional<Trajectory<SwerveSample>> trenchToNeutral = Choreo.loadTrajectory(
                    "TrenchToNeutralLeft"
                );
        Optional<Trajectory<SwerveSample>> neutralMaxBackup = Choreo.loadTrajectory(
                    "NeutralMaxBackupLeft"
                );
        Optional<Trajectory<SwerveSample>> neutralMaxCollect = Choreo.loadTrajectory(
                    "NeutralMaxCollectLeft"
                );
        Optional<Trajectory<SwerveSample>> neutralToTrenchFirst = Choreo.loadTrajectory(
                    "NeutralToTrenchLeftFirst"
                );
        Optional<Trajectory<SwerveSample>> neutralToTrenchSecond = Choreo.loadTrajectory(
            "NeutralToTrenchLeftSecond"
        );
        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);

        addCommands(
            Commands.parallel(
                new WaitUntilCommand(() -> intake.getAngle().getDegrees() < IntakeConstants.STOW_ANGLE.getDegrees() + 2.0)
                    .andThen(aim),
                Commands.sequence(
                    IntakeCommands.deploy(intake),
                    new FollowPath(trenchToNeutralBeach.get(), drivetrain, true, mirror),
                    IntakeCommands.feed(intake),
                    new FollowPath(neutralToTrenchFirst.get(), drivetrain, false, mirror),
                    new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0),
                    IntakeCommands.deploy(intake),
                    new FollowPath(trenchToNeutral.get(), drivetrain, false, mirror),
                    new FollowPath(neutralMaxCollect.get(), drivetrain, false, mirror),
                    IntakeCommands.feed(intake),
                    new FollowPath(neutralMaxBackup.get(), drivetrain, false, mirror),
                    new FollowPath(neutralToTrenchSecond.get(), drivetrain, false, mirror),
                    new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}