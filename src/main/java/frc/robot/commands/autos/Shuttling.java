package frc.robot.commands.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.ShotCalculator;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Shuttling extends SequentialCommandGroup {
    public Shuttling(
        Drivetrain drivetrain,
        Hood hood,
        Flywheel flywheel,
        Feeder feeder,
        Turret turret,
        Intake intake,
        ShotCalculator shotCalculator,
        boolean mirror
    ) {
        Optional<Trajectory<SwerveSample>> shuttlingLeft = Choreo.loadTrajectory(
                    "ShuttlingLeft"
                );

        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);
        addCommands(
            Commands.parallel(
                new WaitUntilCommand(() -> intake.getAngle().getDegrees() < IntakeConstants.STOW_ANGLE.getDegrees() + 2.0)
                    .andThen(aim),
                Commands.sequence(
                    new WaitCommand(1.0),
                    new WaitUntilCommand(() -> FieldUtil.inNuetralZone(drivetrain.poseEstimator.getEstimatedPose().getX())),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot),
                    IntakeCommands.deploy(intake)
                ),
                new FollowPath(shuttlingLeft.get(), drivetrain, true, mirror)
            )
        );

        addRequirements(drivetrain, hood, flywheel, feeder, turret, intake);
    }
}