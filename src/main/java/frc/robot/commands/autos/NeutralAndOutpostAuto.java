package frc.robot.commands.autos;

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

public class NeutralAndOutpostAuto extends SequentialCommandGroup {
    public NeutralAndOutpostAuto(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Intake intake, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> trenchToNeutral = Choreo.loadTrajectory(
                    "TrenchToNeutralRight"
                );
        Optional<Trajectory<SwerveSample>> trenchToOutpost = Choreo.loadTrajectory(
                    "TrenchToOutpost"
                );
        Optional<Trajectory<SwerveSample>> neutralToTrench = Choreo.loadTrajectory(
                    "NeutralToTrenchRight"
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
                    new FollowPath(trenchToOutpost.get(), drivetrain, false),
                    new WaitCommand(2.0),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}