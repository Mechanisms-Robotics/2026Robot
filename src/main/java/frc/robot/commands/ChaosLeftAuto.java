package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChaosLeftAuto extends SequentialCommandGroup {
    public ChaosLeftAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> chaosLeftBackup = Choreo.loadTrajectory(
                    "ChaosLeftBackup"
                );
        Optional<Trajectory<SwerveSample>> chaosLeftChaos = Choreo.loadTrajectory(
                    "ChaosLeftChaos"
                );


        addCommands(
            new FollowPath(chaosLeftBackup.get(), drivetrain, true),
            new WaitCommand(2.0), // simulate shooting
            new FollowPath(chaosLeftChaos.get(), drivetrain, false)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}