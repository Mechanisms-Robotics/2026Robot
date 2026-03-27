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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DepotScoringAuto extends SequentialCommandGroup {
    public DepotScoringAuto(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Intake intake, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> hubBackup = Choreo.loadTrajectory(
                    "HubBackup"
                );
        Optional<Trajectory<SwerveSample>> hubToDepot = Choreo.loadTrajectory(
                    "HubToDepot"
                );

            Optional<Trajectory<SwerveSample>> depotForward = Choreo.loadTrajectory(
                "DepotForward"
            );
        Aim aim = new Aim(flywheel, turret, shotCalculator, poseEstimator);

        addCommands(
            Commands.parallel(
                aim,
                Commands.sequence(
                    IntakeCommands.deploy(intake),
                    Commands.parallel(
                        new FollowPath(hubBackup.get(), drivetrain, true),
                        new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0)
                    ),
                    new FollowPath(hubToDepot.get(), drivetrain, false),
                    new WaitCommand(1),
                    IntakeCommands.feed(intake),
                    new FollowPath(depotForward.get(), drivetrain, false),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}