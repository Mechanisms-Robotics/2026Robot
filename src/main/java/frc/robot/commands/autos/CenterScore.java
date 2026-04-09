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


        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);

        addCommands(
            new WaitUntilCommand(() -> intake.getAngle().getDegrees() < IntakeConstants.STOW_ANGLE.getDegrees() + 2.0)
                .andThen(aim),
            Commands.sequence(
                new FollowPath(backup.get(), drivetrain, true, false, true),
                new WaitCommand(3),
                new ShootCommands.Shoot(feeder, hood, aim::getShot)
            )
        );

        addRequirements(drivetrain, flywheel, feeder, turret);
    }
}