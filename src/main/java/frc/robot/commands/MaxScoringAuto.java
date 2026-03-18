package frc.robot.commands;

import java.lang.StackWalker.Option;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.ShotCalculator;
import frc.robot.PoseEstimator8736;
import frc.robot.util.FieldUtil;
import frc.robot.commands.ShootCommands.Aim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MaxScoringAuto extends SequentialCommandGroup {
    public MaxScoringAuto(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Intake intake, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> trenchToNeutral = Choreo.loadTrajectory(
                    "TrenchToNeutralRight"
                );
        Optional<Trajectory<SwerveSample>> neutralMaxBackup = Choreo.loadTrajectory(
                    "NeutralMaxBackup"
                );
        Optional<Trajectory<SwerveSample>> neutralMaxCollect = Choreo.loadTrajectory(
                    "NeutralMaxCollect"
                );
        Optional<Trajectory<SwerveSample>> neutralToTrench = Choreo.loadTrajectory(
                    "NeutralToTrenchRight"
                );
        final Command intakeCommand = IntakeCommands.intake(intake);
        Aim aim = new Aim(hood, flywheel, turret, shotCalculator, poseEstimator, FieldUtil.getHub().toPose2d());

        addRequirements(drivetrain, flywheel, feeder, intake, turret);


        addCommands(
            aim.withTimeout(0),
            intakeCommand,
            new FollowPath(trenchToNeutral.get(), drivetrain, true),
            new FollowPath(neutralToTrench.get(), drivetrain, false),
            // aim,
            new ShootCommands.Shoot(feeder).withTimeout(3.0),
            new FollowPath(trenchToNeutral.get(), drivetrain, false),
            new FollowPath(neutralMaxCollect.get(), drivetrain, false),
            new FollowPath(neutralMaxBackup.get(), drivetrain, false),
            new FollowPath(neutralToTrench.get(), drivetrain, false),
            // aim,
            new ShootCommands.Shoot(feeder).withTimeout(3.0)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}