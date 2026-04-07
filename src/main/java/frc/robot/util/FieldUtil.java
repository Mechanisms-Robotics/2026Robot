package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    
    public static boolean inAllianceZone(double robotX) {
        return getAlliance().equals(Alliance.Blue) ?
            robotX < FieldConstants.BLUE_ALLIANCE_ZONE
            : robotX > FieldConstants.RED_ALLIANCE_ZONE;
    }

    public static Pose3d getHub() {
        return getAlliance().equals(Alliance.Blue)
            ? FieldConstants.Hub.CENTER_BLUE_POSE : FieldConstants.Hub.CENTER_RED_POSE;
    }

    public static Pose2d getShuttlePose(double robotY) {
        if (getAlliance().equals(Alliance.Blue))
            return robotY > FieldConstants.CENTER.getY()
                ? FieldConstants.SHUTTLE_DEPOT_BLUE_POSE
                : FieldConstants.SHUTTLE_OUTPOST_BLUE_POSE;
        return robotY > FieldConstants.CENTER.getY()
            ? FieldConstants.SHUTTLE_OUTPOST_RED_POSE
            : FieldConstants.SHUTTLE_DEPOT_RED_POSE;
    }

    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.getX(), FieldConstants.WIDTH - pose.getY(), pose.getRotation().unaryMinus());
    }
}