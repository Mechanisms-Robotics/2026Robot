package frc.robot.commands.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.PoseEstimator8736;
import frc.robot.ShotCalculator;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import frc.robot.commands.ShootCommands.ManualShoot;
import frc.robot.commands.ShootCommands.Shoot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShuttlingRight extends SequentialCommandGroup {
    public ShuttlingRight(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> shuttlingRight = Choreo.loadTrajectory(
                    "ShuttlingRight"
                );

        Aim aim = new Aim(flywheel, turret, shotCalculator, poseEstimator);
        addCommands(
            Commands.parallel(
                aim,
                Commands.sequence(
                        Commands.waitSeconds(0.8),
                        new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(15.0)
                ),
                new FollowPath(shuttlingRight.get(), drivetrain, true)
            )
        );

        addRequirements(drivetrain, hood, flywheel, feeder, turret);
    }
}