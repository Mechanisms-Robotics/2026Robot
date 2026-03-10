package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
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
        // TODO: add shooter and other requirements here
    }
}