package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChaosRightAuto extends SequentialCommandGroup {
    public ChaosRightAuto(Drivetrain drivetrain) {
        Optional<Trajectory<SwerveSample>> chaosRightBackup = Choreo.loadTrajectory(
                    "ChaosRightBackup"
                );
        Optional<Trajectory<SwerveSample>> chaosRightChaos = Choreo.loadTrajectory(
                    "ChaosRightChaos"
                );


        addCommands(
            new FollowPath(chaosRightBackup.get(), drivetrain, true),
            new WaitCommand(2.0), // simulate shooting
            new FollowPath(chaosRightChaos.get(), drivetrain, false)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}