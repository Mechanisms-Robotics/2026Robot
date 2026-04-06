package frc.robot.commands.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BeachRightAuto extends SequentialCommandGroup {
    public BeachRightAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> beachOutpost = Choreo.loadTrajectory(
                    "BeachOutpost"
                );

        addCommands(
            new FollowPath(beachOutpost.get(), drivetrain, true)
        );

        addRequirements(drivetrain);
    }
}