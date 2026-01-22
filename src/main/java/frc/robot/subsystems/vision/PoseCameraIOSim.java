package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.CONSTANTS;

public class PoseCameraIOSim implements PoseCameraIO {
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;

    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraToRobot;

    private final PhotonPoseEstimator photonEstimator;
    // The actual simulated position of the robot
    private final Supplier<Pose2d> simPose;

    public PoseCameraIOSim(String cameraName, Transform3d cameraToRobot, Supplier<Pose2d> simPose) {
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        this.simPose = simPose;

        this.visionSim = new VisionSystemSim("visionSim");
        this.visionSim.addAprilTags(CONSTANTS.APRILTAG_FIELD_LAYOUT);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.17, 0.05);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(60);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        this.camera = new PhotonCamera(cameraName);
        this.cameraSim = new PhotonCameraSim(camera, cameraProp);

        this.visionSim.addCamera(cameraSim, cameraToRobot);

        this.photonEstimator = new PhotonPoseEstimator(
            CONSTANTS.APRILTAG_FIELD_LAYOUT, 
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            this.cameraToRobot);

        this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(PoseCameraIOInputs inputs) {
        inputs.isConnected = true;

        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

        List<Double> timestampSecondsArray = new ArrayList<>();
        List<Pose2d> poseEstimatesArray = new ArrayList<>();

        for (PhotonPipelineResult result : results) {
            visionEstimate = this.photonEstimator.update(result);    

            if (visionEstimate.isPresent()) {
                Pose3d poseEstimate = visionEstimate.get().estimatedPose;

                Logger.recordOutput(cameraName + "/simPose", poseEstimate);

                // Push each unread input to the arrays
                timestampSecondsArray.add(visionEstimate.get().timestampSeconds);
                poseEstimatesArray.add(poseEstimate.toPose2d());
            }
        }

        // Finally, push all estimates to the inputs
        inputs.timestampSeconds = timestampSecondsArray.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.poseEstimates = poseEstimatesArray.stream().toArray(Pose2d[]::new);

        visionSim.update(simPose.get());
    }
}