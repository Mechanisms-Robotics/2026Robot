package frc.robot.commands.autos;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;
import frc.robot.ShotCalculator;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MinScoring extends SequentialCommandGroup {
    public MinScoring(
        Drivetrain drivetrain, 
        Hood hood,
        Flywheel flywheel,
        Feeder feeder,
        Turret turret,
        Intake intake,
        ShotCalculator shotCalculator,
        boolean mirror
    ) {
        Optional<Trajectory<SwerveSample>> trenchToNeutral = Choreo.loadTrajectory(
                    "TrenchToNeutralLeft"
                );
        Optional<Trajectory<SwerveSample>> neutralMaxBackup = Choreo.loadTrajectory(
                    "NeutralMaxBackupLeft"
                );
        Optional<Trajectory<SwerveSample>> neutralMaxCollect = Choreo.loadTrajectory(
                    "NeutralMaxCollectLeft"
                );
        Optional<Trajectory<SwerveSample>> neutralToTrench = Choreo.loadTrajectory(
                    "NeutralToTrenchLeft"
                );

        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);

        // Since we shoot preload before a path, we manually reset the pose
        Pose2d startPose = trenchToNeutral
            .get()
            .getInitialPose(FieldUtil.getAlliance().equals(Alliance.Red))
            .get();
            
        if (mirror) {
            FieldUtil.flipPose(startPose);
        }

        addCommands(
            Commands.parallel(
                aim,
                Commands.sequence(
                    new InstantCommand(() -> drivetrain.resetPose(startPose)),
                    new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                    // shoot preload
                    IntakeCommands.deploy(intake),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0),

                    // score first round
                    new FollowPath(trenchToNeutral.get(), drivetrain, false, mirror),
                    IntakeCommands.feed(intake),
                    new FollowPath(neutralToTrench.get(), drivetrain, false, mirror),
                    new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0),

                    // score second round
                    IntakeCommands.deploy(intake),
                    new FollowPath(trenchToNeutral.get(), drivetrain, false, mirror),
                    new FollowPath(neutralMaxCollect.get(), drivetrain, false, mirror),
                    IntakeCommands.feed(intake),
                    new FollowPath(neutralMaxBackup.get(), drivetrain, false, mirror),
                    new FollowPath(neutralToTrench.get(), drivetrain, false, mirror),
                    new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}