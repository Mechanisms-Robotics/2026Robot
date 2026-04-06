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
import frc.robot.CONSTANTS.FieldConstants;
import frc.robot.CONSTANTS.VisionConstants;

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
            FieldConstants.APRILTAG_FIELD_LAYOUT,
            this.cameraToRobot);
    }

    @Override
    public void updateInputs(PoseCameraIOInputs inputs) {
        inputs.isConnected = camera.isConnected();

        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

        List<Double> timestampSecondsArray = new ArrayList<>();
        List<Pose2d> poseEstimatesArray = new ArrayList<>();
        List<List<Double>> aprilTagsDistancesArray = new ArrayList<>();

        for (int i = 0; i < results.size(); i++) {
            PhotonPipelineResult result = results.get(i);
            visionEstimate = this.photonEstimator.estimateCoprocMultiTagPose(result); 

            // if there's no multi-tag estimate, fall back to the lowest ambiguity single tag pose
            if (visionEstimate.isEmpty()) {
                visionEstimate = this.photonEstimator.estimateLowestAmbiguityPose(result);
            }

            if (visionEstimate.isPresent()) {
                Pose3d poseEstimate = visionEstimate.get().estimatedPose;

                Logger.recordOutput(cameraName + "/pose", poseEstimate);

                // Push each unread input to the arrays
                timestampSecondsArray.add(visionEstimate.get().timestampSeconds);
                poseEstimatesArray.add(poseEstimate.toPose2d());

                // Save all of the distances from the camera to all of the april tags in this result
                aprilTagsDistancesArray.add(new ArrayList<>());
                int tagCount = visionEstimate.get().targetsUsed.size();
                for (int j = 0; j < tagCount; j++) {
                    double tagDistance = visionEstimate.get().targetsUsed.get(j)
                        .getBestCameraToTarget()
                        .getTranslation()
                        .getNorm();
                    aprilTagsDistancesArray.get(i).add(tagDistance);
                }
            }
        }

        // Hand off data to Vision.java by saving to the inputs
        inputs.timestampSeconds = timestampSecondsArray.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.poseEstimates = poseEstimatesArray.stream().toArray(Pose2d[]::new);
        inputs.aprilTagsDistancesMeters = new double[aprilTagsDistancesArray.size()][32];
        for (int i = 0; i < aprilTagsDistancesArray.size(); i++) {
            inputs.aprilTagsDistancesMeters[i] = aprilTagsDistancesArray.get(i).stream().mapToDouble(Double::doubleValue).toArray();
        }

    }
}