package frc.robot.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStation.Alliance;
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
