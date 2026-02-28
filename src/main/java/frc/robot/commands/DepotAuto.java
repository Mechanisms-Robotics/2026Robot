package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
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


        addCommands(
            new WaitCommand(2.0), // simulate shooting
            new FollowPath(depotBackup.get(), drivetrain, true),
            new WaitCommand(1.0), // probably unneeded but simulate intaking
            new FollowPath(depotForward.get(), drivetrain, false),
            new WaitCommand(3.0) // simulate shooting
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}