// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.CONSTANTS.DriveConstants;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.DriveCommands;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIORedux;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXRedux;

import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.Feeder;

import java.util.Optional;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.PoseCameraIO;
import frc.robot.subsystems.vision.PoseCameraIOPhoton;
import frc.robot.subsystems.vision.PoseCameraIOSim;

public class RobotContainer {

    private final Drivetrain drivetrain;
    private final Vision vision;
    private final DrivetrainController drivetrainController;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final Feeder feeder; 

    private final CommandPS4Controller controller = new CommandPS4Controller(
        CONSTANTS.CONTROLLER_PORT
    );

    public RobotContainer() {
        // TODO: Think about where to initialize all of this properly
        if (CONSTANTS.CURRENT_MODE == CONSTANTS.SIM_MODE) {
            this.drivetrain = new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FRONT_LEFT),
                new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                new ModuleIOSim(DriveConstants.BACK_LEFT),
                new ModuleIOSim(DriveConstants.BACK_RIGHT)
            );
            this.feeder = new Feeder(
                new FeederIOSim(),
                new FeederIOSim()
            );

            this.vision = new Vision(
                this.drivetrain.poseEstimator,
                new PoseCameraIOSim(
                    "Photon_Camera_Sim1", 
                    Transform3d.kZero, 
                    drivetrain.poseEstimator
                ));

        } else {
            this.drivetrain = new Drivetrain(
                new GyroIORedux(),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_RIGHT)
        
            );
            this.feeder = new Feeder(
                // Instantiate TalonFX-based feeder IO with explicit CAN IDs for the motors.
                new FeederIOTalonFX(
                    CONSTANTS.KICKER_MOTOR_CAN_ID
                ),
                new FeederIOTalonFX(
                    CONSTANTS.SPINDEXER_MOTOR_CAN_ID
                )
            );

            // TODO: move this to the proper constants file (in src/config/constants)
            final String photonCameraName = "Photon_Camera1";
           
            this.vision = new Vision(
                this.drivetrain.poseEstimator,
                new PoseCameraIOPhoton(photonCameraName, Transform3d.kZero)
            );
        }

        this.drivetrainController = new DrivetrainController(this.drivetrain);

        configureBindings();
        generateAutos();
    }

    private void configureBindings() {
        this.controller
            .cross()
            .onTrue(
                new InstantCommand(() -> {
                    this.drivetrain.zeroGyro();
                })
            );

        this.drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    double forward = -this.controller.getLeftY(); // Negative to match FRC convention
                    double strafe = -this.controller.getLeftX();
                    Translation2d driveSpeeds = getDriveVelocity(
                        forward,
                        strafe
                    );
                    double rotation;
                    if (CONSTANTS.CURRENT_MODE == CONSTANTS.Mode.SIM) {
                        rotation = -this.controller.getRawAxis(3); // Why is sim different then driverstation?
                    } else {
                        rotation = -this.controller.getRightX();
                    }

                    // apply deadbands and scaling
                    rotation = MathUtil.applyDeadband(
                        rotation,
                        CONSTANTS.DriveConstants.DEADBAND
                    );

                    rotation = Math.copySign(rotation * rotation, rotation);

                    ChassisSpeeds speeds = new ChassisSpeeds(
                        driveSpeeds.getX() *
                            CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS.in(
                                MetersPerSecond
                            ),
                        driveSpeeds.getY() *
                            CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS.in(
                                MetersPerSecond
                            ),
                        (rotation *
                                (CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS.in(
                                        MetersPerSecond
                                    ))) /
                            CONSTANTS.DriveConstants.DRIVE_BASE_RADIUS
                    );

                    // convert to robot-oriented coordinates and pass to swerve subsystem
                    ChassisSpeeds robotOriented =
                        this.drivetrainController.fieldToRobotChassisSpeeds(
                            speeds
                        );
                    this.drivetrain.setDesiredState(robotOriented);
                },
                this.drivetrain
            )
        );

         this.controller
            .square()
            .onTrue(
                new InstantCommand(() -> {
                    // Apply 12V to feeder motors when the square button is pressed
                    this.feeder.startFeeding();
                }, this.feeder).withName("StartFeeding")
            );
        
        this.controller
            .triangle()
            .onTrue( 
                new InstantCommand(() -> {
                    // Apply 12V to feeder motors when the triangle button is pressed
                    this.feeder.reverseFeeding();
                }, this.feeder).withName("ReverseFeeding")
            );

        this.controller
            .circle()
            .onTrue(
                new InstantCommand(() -> {
                    // Apply 12V to feeder motors when the circle button is pressed
                    this.feeder.stopFeeding();
                }, this.feeder).withName("StopFeeding")
            );
    }

    private void generateAutos() {
        // Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(
        //     "Test Path"
        // );

        autoChooser.setDefaultOption("Wheel Characterization", DriveCommands.wheelRadiusCharacterization(drivetrain));
        autoChooser.addOption("Drive Feedforward Characterization", DriveCommands.feedforwardCharacterization(drivetrain));
        // autoChooser.addOption("Test Path", new FollowPath(trajectory.get(), this.drivetrain, true));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private static Translation2d getDriveVelocity(double x, double y) {
        double linearMag = MathUtil.applyDeadband(
            Math.hypot(x, y),
            DriveConstants.DEADBAND
        );
        Rotation2d direction = new Rotation2d(Math.atan2(y, x));
        linearMag = linearMag * linearMag;
        return new Pose2d(Translation2d.kZero, direction)
            .transformBy(new Transform2d(linearMag, 0.0, Rotation2d.kZero))
            .getTranslation();
    }
}
