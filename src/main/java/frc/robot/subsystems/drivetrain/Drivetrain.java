package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.PoseEstimator8736;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

    // Per WPILib documentation +X is forward and +Y is left (oriented to the robot)
    // Positive rotation is counterclockwise

    SwerveDriveKinematics kinematics;
    ChassisSpeeds desiredChassisSpeeds;
    PoseEstimator8736 poseEstimator;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final GyroIO gyroIO; // may be a real gyro or a simulated gyro
    private final GyroIOInputsAutoLogged gyroInputs =
        new GyroIOInputsAutoLogged();

    public static final Lock odometryLock = new ReentrantLock();

    /**
     * Remember that the forward direction of the robot is +X and the left direction is
     * +Y. So if you give a ChassisSpeed of (+1, +1) the robot should translate diagonally
     * forward and left. This also means that the swerve module positions are based
     * on this coordinate system.
     */

    public Drivetrain(
        GyroIO gyroIO,
        ModuleIO frontLeftModuleIO,
        ModuleIO frontRightModuleIO,
        ModuleIO backLeftModuleIO,
        ModuleIO backRightModuleIO
    ) {
        this.gyroIO = gyroIO;

        this.frontLeftModule = new SwerveModule(
            frontLeftModuleIO,
            "Front Left"
        );
        this.frontRightModule = new SwerveModule(
            frontRightModuleIO,
            "Front Right"
        );
        this.backLeftModule = new SwerveModule(
            backLeftModuleIO, 
            "Back Left"
        );
        this.backRightModule = new SwerveModule(
            backRightModuleIO,
            "Back Right"
        );

        this.kinematics = new SwerveDriveKinematics(
            new Translation2d(
                DriveConstants.FRONT_LEFT.LocationX,
                DriveConstants.FRONT_LEFT.LocationY
            ),
            new Translation2d(
                DriveConstants.FRONT_RIGHT.LocationX,
                DriveConstants.FRONT_RIGHT.LocationY
            ),
            new Translation2d(
                DriveConstants.BACK_LEFT.LocationX,
                DriveConstants.BACK_LEFT.LocationY
            ),
            new Translation2d(
                DriveConstants.BACK_RIGHT.LocationX,
                DriveConstants.BACK_RIGHT.LocationY
            )
        );

        /* TODO: Ask Alex to add comments about the odometry thread when we do a code review. */
        PhoenixOdometryThread.getInstance().start();

        this.poseEstimator = new PoseEstimator8736(
            this.kinematics,
            Rotation2d.kZero,
            Pose2d.kZero
        );

        this.desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    public void setDesiredState(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }

    public ChassisSpeeds getDesiredState() {
        return this.desiredChassisSpeeds;
    }

    public SwerveDriveKinematics getKinematics() {
        return this.kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeftModule.getModulePosition(),
            this.frontRightModule.getModulePosition(),
            this.backLeftModule.getModulePosition(),
            this.backRightModule.getModulePosition(),
        };
    }

    @Override
    public void periodic() {
        Drivetrain.odometryLock.lock();

        // gyroIO may be a real gyro or it may be simulated.
        // These lines update gyroInputs from gyroIO
        // gyroInputs will be used below
        this.gyroIO.updateInputs(this.gyroInputs);
        Logger.processInputs("Drive/Gyro", this.gyroInputs);

        // calling SwerveModule.periodic() updates each module's inputs from the hardware or simulation
        this.frontLeftModule.periodic();
        this.frontRightModule.periodic();
        this.backLeftModule.periodic();
        this.backRightModule.periodic();

        // Send updated odometry to the pose estimator
        // All signals are sampled together so only need timestamps from one
        double[] sampleTimestamps =
            this.frontLeftModule.getOdometryTimestamps();
        for (int i = 0; i < sampleTimestamps.length; i++) {
            // Read wheel positions from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            modulePositions[0] = this.frontLeftModule.getOdometryPositions()[i];
            modulePositions[1] = this.frontRightModule.getOdometryPositions()[i];
            modulePositions[2] = this.backLeftModule.getOdometryPositions()[i];
            modulePositions[3] = this.backRightModule.getOdometryPositions()[i];

            // Update pose estimator with gyro rotation (or null to use kinematics only)
            Rotation2d gyroRotation = this.gyroInputs.connected
                ? this.gyroInputs.odometryYawPositions[i]
                : null;
            this.poseEstimator.updateOdometry(
                modulePositions,
                gyroRotation,
                sampleTimestamps[i]
            );
        }

        // send the new desired states down to the modules

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(
            this.desiredChassisSpeeds,
            CONSTANTS.ROBOT_LOOP_PERIOD
        );

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
            this.desiredChassisSpeeds
        );
        
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS
        );

        Logger.recordOutput("SwerveStates/Setpoints", moduleStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        this.frontLeftModule.setModuleState(frontLeftState);
        this.frontRightModule.setModuleState(frontRightState);
        this.backLeftModule.setModuleState(backLeftState);
        this.backRightModule.setModuleState(backRightState);
    }

    public void zeroGyro() {
        this.resetPose(
            new Pose2d(
                this.poseEstimator.getEstimatedPose().getTranslation(),
                Rotation2d.kZero
            )
        );
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose, getModulePositions());
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPose();
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
