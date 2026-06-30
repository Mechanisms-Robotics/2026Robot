package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.CONSTANTS.FieldConstants;

public class FieldUtil {
    public static Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        }
        
        return Alliance.Red;
    }

    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.getX(), FieldConstants.WIDTH - pose.getY(), pose.getRotation().unaryMinus());
    }
}
