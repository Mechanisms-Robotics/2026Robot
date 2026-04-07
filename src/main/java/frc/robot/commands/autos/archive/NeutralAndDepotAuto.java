package frc.robot.commands.autos.archive;

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
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import frc.robot.commands.ShootCommands.Shoot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NeutralAndDepotAuto extends SequentialCommandGroup {
    public NeutralAndDepotAuto(
        Drivetrain drivetrain,
        Hood hood,
        Flywheel flywheel,
        Feeder feeder,
        Intake intake,
        Turret turret,
        ShotCalculator shotCalculator,
        PoseEstimator8736 poseEstimator
    ) {
        Optional<Trajectory<SwerveSample>> trenchToNeutral = Choreo.loadTrajectory(
                    "TrenchToNeutralLeft"
                );
        Optional<Trajectory<SwerveSample>> hairpinDepot = Choreo.loadTrajectory(
                    "HairpinDepot"
                );
        Optional<Trajectory<SwerveSample>> neutralHairpin = Choreo.loadTrajectory(
                    "NeutralHairpin"
                );
        Aim aim = new Aim(flywheel, turret, shotCalculator, poseEstimator);

        addCommands(
            Commands.parallel(
                aim,
                Commands.sequence(
                    IntakeCommands.deploy(intake),
                    new FollowPath(trenchToNeutral.get(), drivetrain, true),
                    IntakeCommands.feed(intake),
                    new FollowPath(neutralHairpin.get(), drivetrain, false),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0),
                    IntakeCommands.deploy(intake),
                    new FollowPath(hairpinDepot.get(), drivetrain, false),
                    new WaitCommand(1.0),
                    IntakeCommands.feed(intake),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}