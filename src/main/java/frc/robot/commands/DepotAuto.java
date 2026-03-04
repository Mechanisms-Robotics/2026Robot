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
import frc.robot.CONSTANTS;
import frc.robot.commands.ShootCommands.ManualShoot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DepotAuto extends SequentialCommandGroup {
    public DepotAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> depotBackup = Choreo.loadTrajectory(
            "DepotBackup"
        );
        Optional<Trajectory<SwerveSample>> depotForward = Choreo.loadTrajectory(
            "DepotForward"
        );
        ManualShoot shooter = new ManualShoot(
            new Flywheel(new FlywheelIO() {}),
            new Feeder(new FeederIO() {}, new FeederIO() {}),
            50
        );


        addCommands(
            new InstantCommand(shooter::initialize).withTimeout(2.0),
            new InstantCommand(() -> shooter.end(true)),
            new FollowPath(depotBackup.get(), drivetrain, true),
            new WaitCommand(1.0), //simulate intaking
            new FollowPath(depotForward.get(), drivetrain, false),
            new InstantCommand(shooter::initialize).withTimeout(2.0),
            new InstantCommand(() -> shooter.end(true))
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}