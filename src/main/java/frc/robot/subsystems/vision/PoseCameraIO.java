package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface PoseCameraIO {
    @AutoLog
    public static class PoseCameraIOInputs {
        public boolean isConnected = false;

        public double[] timestampSeconds = new double[] {};
        public Pose2d[] poseEstimates = new Pose2d[] {};
    }

    public default void updateInputs(PoseCameraIOInputs inputs) {}
}