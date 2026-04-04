package frc.robot.commands.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BeachLeftAuto extends SequentialCommandGroup {
    public BeachLeftAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> beachDepot = Choreo.loadTrajectory(
                    "BeachDepot"
                );

        addCommands(
            new FollowPath(beachDepot.get(), drivetrain, true)
        );

        addRequirements(drivetrain);
    }
}