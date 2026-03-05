package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.CONSTANTS;
import frc.robot.commands.ShootCommands.ManualShoot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChaosDepotAuto extends SequentialCommandGroup {
    public ChaosDepotAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> chaosLeftBackup = Choreo.loadTrajectory(
            "ChaosDepotBackup"
        );
        Optional<Trajectory<SwerveSample>> chaosLeftChaos = Choreo.loadTrajectory(
            "ChaosDepotChaos"
        );
        ManualShoot shooter = new ManualShoot(
            new Flywheel(new FlywheelIO() {}),
            new Feeder(new FeederIO() {}, new FeederIO() {}),
            100
        );

        Hood hood = new Hood(new HoodIO() {});
        Flywheel flywheel = new Flywheel(new FlywheelIO() {});
        Feeder feeder = new Feeder(new FeederIO() {}, new FeederIO() {});

        addCommands(
            new FollowPath(chaosLeftBackup.get(), drivetrain, true),
            new InstantCommand(hood::stow, hood),
            new ManualShoot(flywheel, feeder, 100).withTimeout(2.0),
            // new WaitCommand(2.0),
            // new InstantCommand(() -> shooter.end(false)),
            new ShootCommands().Shoot(feeder).withTimeout(4.0),
            new FollowPath(chaosLeftChaos.get(), drivetrain, false)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}