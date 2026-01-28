package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Utility class for managing swerve drive pose estimation. Encapsulates the SwerveDrivePoseEstimator
 * and handles odometry updates, vision measurements, and gyro integration.
 */
public class PoseEstimator8736 {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDrivePoseEstimator simulatedPoseEstimator; // only use in simulation

    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
        };

    /**
     * Creates a new PoseEstimator.
     *
     * @param kinematics The swerve drive kinematics
     * @param initialGyroRotation The initial gyro rotation
     * @param initialPose The initial pose estimate
     */
    public PoseEstimator8736(
        SwerveDriveKinematics kinematics,
        Rotation2d initialGyroRotation,
        Pose2d initialPose
    ) {
        this.kinematics = kinematics;
        this.rawGyroRotation = initialGyroRotation;
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            initialPose
        );

        this.simulatedPoseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            initialPose
        );
    }

    /**
     * Updates the pose estimator with odometry data.
     *
     * @param modulePositions The current module positions
     * @param gyroRotation The current gyro rotation (or null to use kinematics)
     * @param timestamp The timestamp of this sample
     */
    public void updateOdometry(
        SwerveModulePosition[] modulePositions,
        Rotation2d gyroRotation,
        double timestamp
    ) {
        // Calculate module deltas
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters -
                    lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle
            );
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroRotation != null) {
            // Use the real gyro angle
            rawGyroRotation = gyroRotation;
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(
                new Rotation2d(twist.dtheta)
            );
        }

        // Apply update to pose estimator
        poseEstimator.updateWithTime(
            timestamp,
            rawGyroRotation,
            modulePositions
        );

        this.simulatedPoseEstimator.updateWithTime(
            timestamp,
            rawGyroRotation,
            modulePositions
        );
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionRobotPoseMeters The vision-measured robot pose
     * @param timestampSeconds The timestamp of the vision measurement
     * @param visionMeasurementStdDevs The standard deviations of the vision measurement
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds,
            visionMeasurementStdDevs
        );
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionRobotPoseMeters The vision-measured robot pose
     * @param timestampSeconds The timestamp of the vision measurement
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds
    ) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds
        );
    }

    /**
     * Resets the pose estimator to a specific pose.
     *
     * @param pose The new pose
     * @param modulePositions The current module positions
     */
    public void resetPose(Pose2d pose, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose);
    }

    /**
     * Returns the current estimated pose.
     *
     * @return The current pose estimate
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Only use this in simulation to get the actual simulated position of the robot.
     * This position is determined by odometry.
     * 
     * @return actual simulated position
     */
    @AutoLogOutput(key = "Simulation/ActualPosition")
    public Pose2d getSimulatedPose() {
        return this.simulatedPoseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current raw gyro rotation.
     *
     * @return The raw gyro rotation
     */
    public Rotation2d getRawGyroRotation() {
        return rawGyroRotation;
    }
}