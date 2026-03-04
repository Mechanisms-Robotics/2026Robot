package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.commands.ShootCommands.ManualShoot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChaosLeftAuto extends SequentialCommandGroup {
    public ChaosLeftAuto(Drivetrain drivetrain, Feeder feeder, Flywheel flywheel, Hood hood) {
        Optional<Trajectory<SwerveSample>> chaosLeftBackup = Choreo.loadTrajectory(
            "ChaosLeftBackup"
        );
        Optional<Trajectory<SwerveSample>> chaosLeftChaos = Choreo.loadTrajectory(
            "ChaosLeftChaos"
        );

        addCommands(
            new FollowPath(chaosLeftBackup.get(), drivetrain, true),
            new InstantCommand(hood::stow, hood),
            new ManualShoot(flywheel, feeder, 100)
                .withTimeout(2.0),
            new ShootCommands.Shoot(feeder)
                .withTimeout(4.0),
            new FollowPath(chaosLeftChaos.get(), drivetrain, false)
        );

        addRequirements(drivetrain, feeder, flywheel, hood);
    }
}