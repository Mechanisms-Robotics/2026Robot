package frc.robot.commands.autos;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

public class LaSiesta extends ParallelCommandGroup {
    public LaSiesta(
        Drivetrain drivetrain, 
        Hood hood,  
        Flywheel flywheel,
        Feeder feeder,
        Turret turret,
        Intake intake,
        ShotCalculator shotCalculator,
        boolean mirror
    ) {
        Optional<Trajectory<SwerveSample>> laSiesta = Choreo.loadTrajectory(
            "LaSiesta"
        );

        Optional<Trajectory<SwerveSample>> laSiesta2 = Choreo.loadTrajectory(
            "LaSiesta2"
        );

        Optional<Trajectory<SwerveSample>> laSiesta3 = Choreo.loadTrajectory(
            "LaSiesta3"
        );
        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);

        addCommands(
            aim,
            Commands.sequence(
                Commands.parallel(
                    new FollowPath(laSiesta.get(), drivetrain, true, mirror),
                    Commands.sequence(
                        new WaitCommand(0.8),
                        new WaitUntilCommand(() -> FieldUtil.inNuetralZone(drivetrain.poseEstimator.getEstimatedPose().getX())),
                        IntakeCommands.deploy(intake)
                    )
                ),

                new WaitCommand(3),
                new FollowPath(laSiesta2.get(), drivetrain, false, mirror),
                IntakeCommands.feed(intake),
                new FollowPath(laSiesta3.get(), drivetrain, false, mirror),
                new InstantCommand(() -> drivetrain.poseEstimator.setVisionEnabled(true)),
                new ShootCommands.Shoot(feeder, hood, aim::getShot)
            )
        );
        addRequirements(drivetrain, flywheel, feeder, intake, turret);
    }
}
