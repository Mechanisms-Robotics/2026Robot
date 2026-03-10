package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CONSTANTS.ManualModeConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class ManualAutos {
    public static class CenterHubBackup extends SequentialCommandGroup {
        public CenterHubBackup(Drivetrain drivetrain, Flywheel flywheel, Feeder feeder) {
            Optional<Trajectory<SwerveSample>> backupLeftToTower = Choreo.loadTrajectory(
                        "BackupLeftToTower"
                    );
    
            addRequirements(drivetrain, flywheel, feeder);
            addCommands(
                new FollowPath(backupLeftToTower.get(), drivetrain, true),
                new ShootCommands.ManualShoot(flywheel, feeder, ManualModeConstants.FLYWHEEL_RPM).withTimeout(4.0)
            );
        }

    }
    
    public static class OutpostBackup extends SequentialCommandGroup {
        public OutpostBackup(Drivetrain drivetrain, Flywheel flywheel, Feeder feeder) {
            Optional<Trajectory<SwerveSample>> backupLeftToTower = Choreo.loadTrajectory(
                        "OutpostBackupShoot"
                    );
    
            addRequirements(drivetrain, flywheel, feeder);
            addCommands(
                new FollowPath(backupLeftToTower.get(), drivetrain, true),
                new ShootCommands.ManualShoot(flywheel, feeder, ManualModeConstants.FLYWHEEL_RPM).withTimeout(4.0)
            );
        }
    }

    public static class DepotBackup extends SequentialCommandGroup {
        public DepotBackup(Drivetrain drivetrain, Flywheel flywheel, Feeder feeder) {
            Optional<Trajectory<SwerveSample>> backupLeft = Choreo.loadTrajectory(
                        "ShootPreloadLeft"
                    );
    
            addRequirements(drivetrain, flywheel, feeder);
            addCommands(
                new FollowPath(backupLeft.get(), drivetrain, true),
                new ShootCommands.ManualShoot(flywheel, feeder, ManualModeConstants.FLYWHEEL_RPM).withTimeout(4.0)
            );
        }
    }
}
