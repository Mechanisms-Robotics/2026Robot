package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator8736;

public class Vision extends SubsystemBase {
    private final PoseCameraIO[] ios;
    private final PoseCameraIOInputsAutoLogged[] inputs;

    private final PoseEstimator8736 poseEstimator;

    // The cameraName here is used for logging purposes
    public Vision(PoseEstimator8736 poseEstimator, PoseCameraIO... ios) {
        this.ios = ios;
        this.poseEstimator = poseEstimator;
        this.inputs = new PoseCameraIOInputsAutoLogged[ios.length];

        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new PoseCameraIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + inputs[i], inputs[i]);

            // Constantly feed vision measurements into the pose estimator
            for (int j = 0; j < inputs[i].timestampSeconds.length; j++) {
                this.poseEstimator.addVisionMeasurement(inputs[i].poseEstimates[j], inputs[i].timestampSeconds[j]);
            }
        }
    }
}
