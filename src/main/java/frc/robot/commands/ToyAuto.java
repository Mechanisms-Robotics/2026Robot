package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class ToyAuto extends SequentialCommandGroup {
    public ToyAuto(Drivetrain drivetrain, Hood hood, Flywheel flywheel, Feeder feeder, Intake intake, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Optional<Trajectory<SwerveSample>> toyAutoForward = Choreo.loadTrajectory(
                    "ToyAutoForward"
                );
        Optional<Trajectory<SwerveSample>> toyAutoBackwards = Choreo.loadTrajectory(
            "ToyAutoBackwards"
        );
        Aim aim = new Aim(flywheel, turret, shotCalculator, poseEstimator);

        addCommands(
            Commands.parallel(
                aim,
                Commands.sequence(
                    IntakeCommands.deploy(intake),
                    new FollowPath(toyAutoForward.get(), drivetrain, true),
                    IntakeCommands.feed(intake),
                    new FollowPath(toyAutoBackwards.get(), drivetrain, false)
                )
            )
        );

        addRequirements(drivetrain, hood, flywheel, feeder, intake, turret);
    }
}