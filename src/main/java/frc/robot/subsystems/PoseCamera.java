package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;
import frc.robot.PoseEstimator8736;

public class PoseCamera extends SubsystemBase {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraToRobot;

    private PoseEstimator8736 poseEstimator;
    private PhotonPoseEstimator photonEstimator;

    public PoseCamera(String cameraName, Transform3d cameraToRobot, PoseEstimator8736 poseEstimator) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;

        this.poseEstimator = poseEstimator;

        this.photonEstimator = new PhotonPoseEstimator(
            CONSTANTS.APRILTAG_FIELD_LAYOUT, 
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            this.cameraToRobot);

        this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

        for (PhotonPipelineResult result : results) {

            visionEstimate = this.photonEstimator.update(result);        
            if (visionEstimate.isPresent()) {
                Pose3d poseEstimate = visionEstimate.get().estimatedPose;

                SmartDashboard.putNumber(cameraName + "/pose/x", poseEstimate.getX());
                SmartDashboard.putNumber(cameraName + "/pose/y", poseEstimate.getY());
                SmartDashboard.putNumber(cameraName + "/pose/heading", poseEstimate.getRotation().getAngle());
                SmartDashboard.putNumber(cameraName + "/timestamp", visionEstimate.get().timestampSeconds);

                this.poseEstimator.addVisionMeasurement(poseEstimate.toPose2d(),
                    visionEstimate.get().timestampSeconds);
            }
        }
    }
}
