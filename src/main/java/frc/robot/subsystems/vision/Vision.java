package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator8736;
import static frc.robot.CONSTANTS.VisionConstants;

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
            Logger.processInputs("Vision/" + i, inputs[i]);

            // Constantly feed vision measurements into the pose estimator
            for (int j = 0; j < inputs[i].timestampSeconds.length; j++) {
                double totalDistance = 0;
                for (double distance : inputs[i].aprilTagsDistancesMeters[j]) {
                    totalDistance += distance;
                }

                int aprilTagCount = inputs[i].aprilTagsDistancesMeters[j].length;
                double averageDistance = totalDistance / (double) aprilTagCount;
    
                double xyStdDevs =
                    VisionConstants.TRANSLATION_STD_DEV_COEFFICIENT
                        * Math.pow(averageDistance, 1.2)
                        / Math.pow(aprilTagCount, 2.0);
                double thetaStdDev =
                    VisionConstants.ROTATION_STD_DEV_COEFFICIENT
                        * Math.pow(averageDistance, 1.2)
                        / Math.pow(aprilTagCount, 2.0);

                Logger.recordOutput("Vision/stddevs/" + i + "/" + j, xyStdDevs);
                this.poseEstimator.addVisionMeasurement(
                    inputs[i].poseEstimates[j],
                    inputs[i].timestampSeconds[j],
                    VecBuilder.fill(xyStdDevs, xyStdDevs, thetaStdDev)
                );
            }
        }
    }
}
