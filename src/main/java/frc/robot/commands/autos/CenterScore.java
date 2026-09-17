package frc.robot.commands.autos;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.CONSTANTS.ManualModeConstants;
import frc.robot.ShotCalculator;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;

public class CenterScore extends ParallelCommandGroup {
    public CenterScore(
        Drivetrain drivetrain, 
        Hood hood,
        Flywheel flywheel,
        Feeder feeder,
        Turret turret,
        Intake intake,
        ShotCalculator shotCalculator
    ) {
        Optional<Trajectory<SwerveSample>> backup = Choreo.loadTrajectory(
                    "BackupCenter"
                );


        drivetrain.poseEstimator.setVisionEnabled(false);

        ShootCommands.ManualShoot manualShootCommand = new ShootCommands.ManualShoot(
            flywheel,
            feeder,
            ManualModeConstants.FLYWHEEL_RPM
        );

        addCommands(
            Commands.sequence(
                new FollowPath(backup.get(), drivetrain, true, false, false),
                new WaitCommand(1),
                manualShootCommand.withTimeout(3)
            )
        );

        addRequirements(drivetrain, flywheel, feeder, turret);
    }
}