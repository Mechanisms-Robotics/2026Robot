package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface PoseCameraIO {
    @AutoLog
    public static class PoseCameraIOInputs {
        public boolean isConnected = false;

        public double[] timestampSeconds = new double[] {};
        public Pose3d[] poseEstimates = new Pose3d[] {};
        public Pose3d[][] aprilTagPoses = new Pose3d[][] {};
    }

    public default void updateInputs(PoseCameraIOInputs inputs) {}
}