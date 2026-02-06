package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
            drivetrain.getGyroRotation()
        );
    }
}
