package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.CONSTANTS.FieldConstants;
import frc.robot.CONSTANTS.Hub;

public class FieldUtil {
    private static Alliance alliance = null;

    public static Alliance getAlliance() {
        if (alliance != null)
            return alliance;
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
            return alliance;
        }
        return Alliance.Blue;
    }
    
    public static boolean inAllianceZone(Pose2d robot) {
        return getAlliance().equals(Alliance.Blue) ?
            robot.getX() < FieldConstants.BLUE_ALLIANCE_ZONE
            : robot.getX() > FieldConstants.RED_ALLIANCE_ZONE;
    }

    public static Pose3d getHub() {
        return getAlliance().equals(Alliance.Blue)
            ? Hub.CENTER_BLUE_POSE : Hub.CENTER_RED_POSE;
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
}
