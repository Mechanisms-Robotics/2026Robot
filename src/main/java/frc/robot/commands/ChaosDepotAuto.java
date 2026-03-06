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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.ManualModeConstants;
import frc.robot.commands.ShootCommands.ManualShoot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChaosDepotAuto extends SequentialCommandGroup {
    public ChaosDepotAuto(Drivetrain drivetrain, Flywheel flywheel, Feeder feeder, Intake intake) {
        Optional<Trajectory<SwerveSample>> chaosDepotBackup = Choreo.loadTrajectory(
            "ChaosDepotBackup"
        );
        Optional<Trajectory<SwerveSample>> chaosDepotChaos = Choreo.loadTrajectory(
            "ChaosDepotChaos"
        );
        addRequirements(drivetrain, flywheel, feeder, intake);

        addCommands(
            new FollowPath(chaosDepotBackup.get(), drivetrain, true),
            new ShootCommands.ManualShoot(flywheel, feeder, ManualModeConstants.FLYWHEEL_RPM).withTimeout(4.0),
            new FollowPath(chaosDepotChaos.get(), drivetrain, false)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}