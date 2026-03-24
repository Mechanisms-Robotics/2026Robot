package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.ShotCalculator;
import frc.robot.PoseEstimator8736;
import frc.robot.commands.ShootCommands.Aim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NeutralAndHubBackLeftAuto extends SequentialCommandGroup {
    public NeutralAndHubBackLeftAuto(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Intake intake, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> trenchToNeutral = Choreo.loadTrajectory(
                    "TrenchToNeutralLeft"
                );
        Optional<Trajectory<SwerveSample>> trenchToHubBack = Choreo.loadTrajectory(
                    "TrenchToHubBackLeft"
                );
        Optional<Trajectory<SwerveSample>> hubBackToTrench = Choreo.loadTrajectory(
                    "HubBackToTrenchLeft"
                );
        Optional<Trajectory<SwerveSample>> neutralToTrench = Choreo.loadTrajectory(
                    "NeutralToTrenchLeft"
                );
        Aim aim = new Aim(flywheel, turret, shotCalculator, poseEstimator);

        addCommands(
            Commands.parallel(
                aim,
                Commands.sequence(
                    IntakeCommands.deploy(intake),
                    new FollowPath(trenchToNeutral.get(), drivetrain, true),
                    IntakeCommands.feed(intake),
                    new FollowPath(neutralToTrench.get(), drivetrain, false),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0),
                    IntakeCommands.deploy(intake),
                    new FollowPath(trenchToHubBack.get(), drivetrain, false),
                    IntakeCommands.feed(intake),
                    new FollowPath(hubBackToTrench.get(), drivetrain, false),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}