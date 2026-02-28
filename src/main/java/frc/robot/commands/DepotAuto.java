package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DepotAuto extends SequentialCommandGroup {
    public DepotAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> depotBackup = Choreo.loadTrajectory(
                    "DepotBackup"
                );
        addCommands(
          // shoot here with a timeout (see Slack example)
            new FollowPath(depotBackup.get(), drivetrain, true)
        );
    }
}