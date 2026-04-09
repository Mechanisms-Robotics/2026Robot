package frc.robot.commands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.FieldUtil;

public class FollowPath extends Command {

  private final Trajectory<SwerveSample> trajectory;
  private final Drivetrain drivetrain;
  private final boolean resetPose;
  private boolean isRedAlliance;
  private final boolean isMirrored;

  private final Timer timer = new Timer();
  private final HolonomicDriveController holonomicController;
  
  public FollowPath(
      Trajectory<SwerveSample> trajectory,
      Drivetrain drivetrain,
      boolean resetPose,
      boolean isMirrored) {

    this.trajectory = trajectory;
    this.drivetrain = drivetrain;
    this.resetPose = resetPose;
    this.isMirrored = isMirrored;

    Constraints thetaProfile = new TrapezoidProfile.Constraints(
        CONSTANTS.DriveConstants.ANGLE_MAX_ACCELERATION, CONSTANTS.DriveConstants.ANGLE_MAX_ACCELERATION);

    ProfiledPIDController thetaController =
        new ProfiledPIDController(CONSTANTS.PATH_FOLLOWER_P_THETA, 0, 0, thetaProfile);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    holonomicController = new HolonomicDriveController(
        new PIDController(CONSTANTS.PATH_FOLLOWER_P_X, 0, 0),
        new PIDController(CONSTANTS.PATH_FOLLOWER_P_Y, 0, 0),
        thetaController
    );

    super.addRequirements(drivetrain);
  }

  public FollowPath(
    Trajectory<SwerveSample> trajectory,
    Drivetrain drivetrain,
    boolean resetPose) {
    this(trajectory, drivetrain, resetPose, false);
  }


  // ------------------------
  // COMMAND LIFECYCLE (2025)
  // ------------------------

  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    // disable vision updates while following a path
    this.drivetrain.poseEstimator.setVisionEnabled(false);

    this.isRedAlliance = DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Red;

    if (this.resetPose) {
      // rotate the initial pose if we're on the red alliance
      Optional<Pose2d> initialPose = trajectory.getInitialPose(this.isRedAlliance); 

      if (initialPose.isEmpty()) {
        // TODO: Why would this ever happen? Should we handle it differently?
        throw new IllegalStateException("Trajectory has no initial pose!");
      }
      this.drivetrain.resetPose(
        this.isMirrored ? FieldUtil.flipPose(initialPose.get()) : initialPose.get());
    }
  }

  @Override
  public void execute() {
    double t = this.timer.get();

    Optional<SwerveSample> swerveSample = this.trajectory.sampleAt(
      t, isRedAlliance);
    if (swerveSample.isEmpty()) {
      return; // TODO: Why would this ever happen? Should we handle it differently?
    }
    SwerveSample sample;

    if (this.isMirrored) {
      sample = new SwerveSample(
        swerveSample.get().t,
        swerveSample.get().x,
        FieldConstants.WIDTH - swerveSample.get().y,
        -swerveSample.get().heading,
        swerveSample.get().vx,
        -swerveSample.get().vy,
        -swerveSample.get().omega,
        swerveSample.get().ax,
        -swerveSample.get().ay,
        swerveSample.get().alpha,
        swerveSample.get().moduleForcesX(),
        new double[]{
          -swerveSample.get().moduleForcesY()[0],
          -swerveSample.get().moduleForcesY()[1],
          -swerveSample.get().moduleForcesY()[2],
          -swerveSample.get().moduleForcesY()[3]
        }
      );
    } else {
      sample = swerveSample.get();
    }

    // TODO: This loses the capability of Choreo to control the wheels optimally. See the choreo docs.

    // See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html

    ChassisSpeeds sampleSpeeds = sample.getChassisSpeeds();

    double desiredLinearVelocity = Math.sqrt(
        sampleSpeeds.vxMetersPerSecond * sampleSpeeds.vxMetersPerSecond +
        sampleSpeeds.vyMetersPerSecond * sampleSpeeds.vyMetersPerSecond);

    ChassisSpeeds commandedSpeeds =
        this.holonomicController.calculate(
            this.drivetrain.getPose(),
            sample.getPose(),
            desiredLinearVelocity,
            sample.getPose().getRotation()
        );

    this.drivetrain.setDesiredState(commandedSpeeds);  
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.setDesiredState(new ChassisSpeeds()); // TODO: There may be cases where we don't want the robot to stop!
    this.timer.stop();
  }

  @Override
  public boolean isFinished() {
    // TODO: If precition is important we may need an end-state controller.
    return this.timer.get() >= this.trajectory.getTotalTime(); // TODO: I assume total time is in seconds?
  }  
}