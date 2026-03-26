package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChaosRightAuto extends SequentialCommandGroup {
    public ChaosRightAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> chaosRightChaos = Choreo.loadTrajectory(
                    "ChaosRightChaos"
                );

        addCommands(
            new FollowPath(chaosRightChaos.get(), drivetrain, true)
        );

        addRequirements(drivetrain);
    }
}