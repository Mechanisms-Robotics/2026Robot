package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.ShootCommands.ManualShoot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class UFCRight extends SequentialCommandGroup {
    public UFCRight(Drivetrain drivetrain, Feeder feeder, Flywheel flywheel) {
        Optional<Trajectory<SwerveSample>> ufcRight = Choreo.loadTrajectory(
                    "UFCRight"
                );

        addCommands(
            Commands.sequence(
                new FollowPath(ufcRight.get(), drivetrain, true),
                new ManualShoot(flywheel, feeder, 3300).withTimeout(3.0)
            )
        );

        addRequirements(drivetrain, flywheel, feeder);
    }
}