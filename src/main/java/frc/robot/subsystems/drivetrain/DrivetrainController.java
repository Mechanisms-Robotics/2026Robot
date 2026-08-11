package frc.robot.subsystems.drivetrain;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStation.Alliance;

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
