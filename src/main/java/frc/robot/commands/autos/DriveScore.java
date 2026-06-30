package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.CONSTANTS.IntakeConstants;
import frc.robot.ShotCalculator;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

public class DriveScore extends ParallelCommandGroup {
    public DriveScore(
        Drivetrain drivetrain, 
        Hood hood,
        Flywheel flywheel,
        Feeder feeder,
        Turret turret,
        Intake intake,
        ShotCalculator shotCalculator,
        ChassisSpeeds speeds,
        double time
    ) {
        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);

        addCommands(
            new WaitUntilCommand(() -> intake.getAngle().getDegrees() < IntakeConstants.STOW_ANGLE.getDegrees() + 2.0)
                .andThen(aim),
            Commands.sequence(
                new InstantCommand(() -> drivetrain.setDesiredState(speeds)),
                new WaitCommand(time),
                new InstantCommand(() -> drivetrain.setDesiredState(new ChassisSpeeds())),
                new WaitCommand(2.0),
                new ShootCommands.Shoot(feeder, hood, aim::getShot)
            )
        );

        addRequirements(drivetrain, flywheel, feeder, turret);
    }
}