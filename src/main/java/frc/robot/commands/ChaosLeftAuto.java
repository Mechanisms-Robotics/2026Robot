package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChaosLeftAuto extends SequentialCommandGroup {
    public ChaosLeftAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> chaosLeftChaos = Choreo.loadTrajectory(
                    "ChaosLeftChaos"
                );


        addCommands(
            new FollowPath(chaosLeftChaos.get(), drivetrain, true)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}