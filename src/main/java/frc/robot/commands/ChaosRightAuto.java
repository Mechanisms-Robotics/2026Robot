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

public class ChaosRightAuto extends SequentialCommandGroup {
    public ChaosRightAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> chaosRightBackup = Choreo.loadTrajectory(
            "ChaosRightBackup"
        );
        Optional<Trajectory<SwerveSample>> chaosRightChaos = Choreo.loadTrajectory(
            "ChaosRightChaos"
        );
        ManualShoot shooter = new ManualShoot(
            new Flywheel(new FlywheelIO() {}),
            new Feeder(new FeederIO() {}, new FeederIO() {}),
            50
        );


        addCommands(
            new FollowPath(chaosRightBackup.get(), drivetrain, true),
            new InstantCommand(shooter::initialize).withTimeout(2.0),
            new InstantCommand(() -> shooter.end(true)),
            new FollowPath(chaosRightChaos.get(), drivetrain, false)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}