package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.CONSTANTS;
import frc.robot.commands.ShootCommands.ManualShoot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChaosOutpostAuto extends SequentialCommandGroup {
    public ChaosOutpostAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> chaosRightBackup = Choreo.loadTrajectory(
            "ChaosOutpostBackup"
        );
        Optional<Trajectory<SwerveSample>> chaosRightChaos = Choreo.loadTrajectory(
            "ChaosOutpostChaos"
        );
        ManualShoot shooter = new ManualShoot(
            new Flywheel(new FlywheelIO() {}),
            new Feeder(new FeederIO() {}, new FeederIO() {}),
            50
        );

        Hood hood = new Hood(new HoodIO() {});
        Flywheel flywheel = new Flywheel(new FlywheelIO() {});
        Feeder feeder = new Feeder(new FeederIO() {}, new FeederIO() {});

        addCommands(
            new FollowPath(chaosRightBackup.get(), drivetrain, true),
            new ManualShoot(flywheel, feeder, 100).withTimeout(2.0),
            // new WaitCommand(2.0)
            // new InstantCommand(() -> shooter.end(false)),
            new ShootCommands.Shoot(feeder).withTimeout(2.0),
            new FollowPath(chaosRightChaos.get(), drivetrain, false)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}