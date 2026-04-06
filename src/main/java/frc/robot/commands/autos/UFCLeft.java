package frc.robot.commands.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.ManualShoot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class UFCLeft extends SequentialCommandGroup {
    public UFCLeft(Drivetrain drivetrain, Feeder feeder, Flywheel flywheel) {
        Optional<Trajectory<SwerveSample>> ufcLeft = Choreo.loadTrajectory(
                    "UFCLeft"
                );

        addCommands(
            Commands.parallel(
                new FollowPath(ufcLeft.get(), drivetrain, true),
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    new ManualShoot(flywheel, feeder, 3000).withTimeout(4.0)
                )
            )
            
        );

        addRequirements(drivetrain, flywheel, feeder);
    }
}