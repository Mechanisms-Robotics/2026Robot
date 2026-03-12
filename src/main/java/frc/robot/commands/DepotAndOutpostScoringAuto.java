package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DepotAndOutpostScoringAuto extends SequentialCommandGroup {
    public DepotAndOutpostScoringAuto(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Intake intake, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> hubBackup = Choreo.loadTrajectory(
                    "HubBackup"
                );
        // Optional<Trajectory<SwerveSample>> depotForward = Choreo.loadTrajectory(
        //             "DepotForward"
        //         );
        Optional<Trajectory<SwerveSample>> hubToDepot = Choreo.loadTrajectory(
                    "HubToDepot"
                );
        Optional<Trajectory<SwerveSample>> hubToOutpost = Choreo.loadTrajectory(
                    "HubToOutpost"
                );

        final Command intakeCommand = IntakeCommands.intake(intake);
        Aim aim = new Aim(hood, flywheel, turret, shotCalculator, poseEstimator, FieldUtil.getHub().toPose2d());

        addRequirements(drivetrain, flywheel, feeder, intake, turret);


        addCommands(
            new FollowPath(hubBackup.get(), drivetrain, true),
            aim,
            new ShootCommands.Shoot(feeder).withTimeout(null),
            new FollowPath(hubToDepot.get(), drivetrain, false),
            intakeCommand.withTimeout(2.0),
            // new FollowPath(depotForward.get(), drivetrain, false),
            aim,
            new ShootCommands.Shoot(feeder).withTimeout(3.0),
            new FollowPath(hubToOutpost.get(), drivetrain, false),
            new WaitCommand(2.0),
            aim,
            new ShootCommands.Shoot(feeder).withTimeout(3.0)
        );

        addRequirements(drivetrain);
        // TODO: add shooter and other requirements here
    }
}