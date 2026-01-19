package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator8736;

public class PoseCamera extends SubsystemBase {
    private final PoseCameraIO io;
    private final PoseCameraIOInputsAutoLogged inputs = new PoseCameraIOInputsAutoLogged();

    private final String cameraName;
    private final PoseEstimator8736 poseEstimator;

    public PoseCamera(PoseCameraIO io, String cameraName, PoseEstimator8736 poseEstimator) {
        this.io = io;
        this.cameraName = cameraName;
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/" + cameraName, inputs);

        // Constantly feed vision measurements into the pose estimator
        for (int x = 0; x < inputs.timestampSeconds.length; x++) {
            this.poseEstimator.addVisionMeasurement(inputs.poseEstimates[x], inputs.timestampSeconds[x]);
        }

        inputs.timestampSeconds = new double[] {};
        inputs.poseEstimates = new Pose2d[] {};
        
    }
}
