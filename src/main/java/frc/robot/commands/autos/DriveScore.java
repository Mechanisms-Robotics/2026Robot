package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShotCalculator;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.Aim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
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
        ShotCalculator shotCalculator,
        ChassisSpeeds speeds,
        double time
    ) {
        Aim aim = new Aim(flywheel, turret, shotCalculator, drivetrain.poseEstimator);

        addCommands(
            aim,
            Commands.sequence(
                new InstantCommand(() -> drivetrain.setDesiredState(speeds)),
                new WaitCommand(time),
                new InstantCommand(() -> drivetrain.setDesiredState(new ChassisSpeeds())),
                new ShootCommands.Shoot(feeder, hood, aim::getShot)
            )
        );

        addRequirements(drivetrain, flywheel, feeder, turret);
    }
}