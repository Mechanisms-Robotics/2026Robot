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
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Depot extends SequentialCommandGroup {
    public Depot(
        Drivetrain drivetrain, 
        Hood hood,
        Flywheel flywheel,
        Feeder feeder,
        Turret turret,
        Intake intake,
        ShotCalculator shotCalculator,
        boolean mirror
    ) {
        Optional<Trajectory<SwerveSample>> trenchToDepot = Choreo.loadTrajectory(
                    "TrenchToDepotLeft"
                );
        Optional<Trajectory<SwerveSample>> depotOutwards = Choreo.loadTrajectory(
                    "DepotOutwards"
                );

        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);

        // Since we shoot preload before a path, we manually reset the pose
        Pose2d startPose = trenchToDepot
            .get()
            .getInitialPose(FieldUtil.getAlliance().equals(Alliance.Red))
            .get();
            
        if (mirror) {
            FieldUtil.flipPose(startPose);
        }

        addCommands(
            Commands.parallel(
                new WaitUntilCommand(() -> intake.getAngle().getDegrees() < IntakeConstants.STOW_ANGLE.getDegrees() + 2.0)
                    .andThen(aim),
                Commands.sequence(
                    new InstantCommand(() -> drivetrain.resetPose(startPose)),
                    new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                    // shoot preload
                    IntakeCommands.deploy(intake),
                    new WaitCommand(2),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(3.0),

                    // score first round
                    new FollowPath(trenchToDepot.get(), drivetrain, false, mirror),
                    Commands.waitSeconds(1.0),
                    IntakeCommands.feed(intake),
                    new FollowPath(depotOutwards.get(), drivetrain, false, mirror),
                    new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                    new ShootCommands.Shoot(feeder, hood, aim::getShot).withTimeout(5.0)
                )
            )
        );

        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}