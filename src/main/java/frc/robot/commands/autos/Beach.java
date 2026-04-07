package frc.robot.commands.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Beach extends SequentialCommandGroup {
    public Beach(Drivetrain drivetrain, boolean mirror) {
        Optional<Trajectory<SwerveSample>> beachDepot = Choreo.loadTrajectory(
                    "BeachDepot"
                );

        addCommands(
            new FollowPath(beachDepot.get(), drivetrain, true, mirror)
        );

        addRequirements(drivetrain);
    }
}