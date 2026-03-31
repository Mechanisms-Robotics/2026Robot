package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.PoseEstimator8736;
import frc.robot.ShotCalculator;
import frc.robot.commands.ShootCommands.Aim;
import frc.robot.commands.ShootCommands.ManualShoot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlbanyRight extends SequentialCommandGroup {
    public AlbanyRight(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> albanyRight1 = Choreo.loadTrajectory(
                    "AlbanyRight1"
                );
        Optional<Trajectory<SwerveSample>> albanyRight2 = Choreo.loadTrajectory(
            "AlbanyRight2"
        );
        Optional<Trajectory<SwerveSample>> albanyRight3 = Choreo.loadTrajectory(
            "AlbanyRight3"
        );

        Aim aim = new Aim(flywheel, turret, shotCalculator, poseEstimator);
        addCommands(
            Commands.parallel(
                aim,
                Commands.sequence(
                    new FollowPath(albanyRight1.get(), drivetrain, true),
                    Commands.waitSeconds(3),
                    new FollowPath(albanyRight2.get(), drivetrain, false),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(1.5),
                    new FollowPath(albanyRight3.get(), drivetrain, false)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder);
    }
}