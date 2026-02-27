package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DrivetrainController {

    private final Drivetrain drivetrain;

    public DrivetrainController(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public ChassisSpeeds fieldToRobotChassisSpeeds(
        ChassisSpeeds fieldOriented
    ) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldOriented,
            DriverStation.getAlliance().isPresent() &&
            DriverStation.getAlliance().get() == Alliance.Red
                ? drivetrain.getPose().getRotation().rotateBy(Rotation2d.k180deg) : drivetrain.getPose().getRotation()
        );
    }
}   
