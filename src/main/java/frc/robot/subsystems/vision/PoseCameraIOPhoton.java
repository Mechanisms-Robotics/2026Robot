package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.CONSTANTS;

public class PoseCameraIOPhoton implements PoseCameraIO {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraToRobot;

    private PhotonPoseEstimator photonEstimator;

    // The cameraName here is used to identify the camera in network tables
    public PoseCameraIOPhoton(String cameraName, Transform3d cameraToRobot) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;

        this.photonEstimator = new PhotonPoseEstimator(
            CONSTANTS.APRILTAG_FIELD_LAYOUT, 
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            this.cameraToRobot);

        this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(PoseCameraIOInputs inputs) {
        inputs.isConnected = camera.isConnected();

        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

        List<Double> timestampSecondsArray = new ArrayList<>();
        List<Pose2d> poseEstimatesArray = new ArrayList<>();

        for (PhotonPipelineResult result : results) {
            visionEstimate = this.photonEstimator.update(result);    

            if (visionEstimate.isPresent()) {
                Pose3d poseEstimate = visionEstimate.get().estimatedPose;

                Logger.recordOutput(cameraName + "/pose/x", poseEstimate.getX());
                Logger.recordOutput(cameraName + "/pose/y", poseEstimate.getY());
                Logger.recordOutput(cameraName + "/pose/heading", poseEstimate.getRotation().getAngle());
                Logger.recordOutput(cameraName + "/timestamp", visionEstimate.get().timestampSeconds);

                // Push each unread input to the arrays
                timestampSecondsArray.add(visionEstimate.get().timestampSeconds);
                poseEstimatesArray.add(poseEstimate.toPose2d());
            }
        }

        // Finally, push all estimates to the inputs
        inputs.timestampSeconds = timestampSecondsArray.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.poseEstimates = poseEstimatesArray.stream().toArray(Pose2d[]::new);
    }
}