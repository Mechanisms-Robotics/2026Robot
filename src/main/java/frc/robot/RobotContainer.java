// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.CONSTANTS.CAMERA1_NAME;
import static frc.robot.CONSTANTS.CAMERA1_TRANSFORM3D;
import static frc.robot.CONSTANTS.CAMERA2_NAME;
import static frc.robot.CONSTANTS.CAMERA2_TRANSFORM3D;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.CONSTANTS.TurretConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.DepotAuto;
import frc.robot.commands.ChaosRightAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIORedux;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXRedux;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.PoseCameraIOPhoton;
import frc.robot.subsystems.vision.PoseCameraIOSim;

public class RobotContainer {

    public final Drivetrain drivetrain;
    private final Vision vision;
    private final DrivetrainController drivetrainController;
    public final Turret turret;
    public final SendableChooser<String> autoChooser = new SendableChooser<>();

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

            this.vision = new Vision(
                this.drivetrain.poseEstimator,
                new PoseCameraIOSim(
                    "Photon_Camera_Sim1", 
                    Transform3d.kZero, 
                    drivetrain.poseEstimator
                ));

            this.turret = new Turret(
                new TurretIOSim(), 
                TurretConstants.ROBOT_TO_TURRET, 
                this.drivetrain.poseEstimator);

        } else {
            this.drivetrain = new Drivetrain(
                new GyroIORedux(),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_RIGHT)
            );
           
            this.vision = new Vision(
                this.drivetrain.poseEstimator,
                new PoseCameraIOPhoton(CAMERA1_NAME, CAMERA1_TRANSFORM3D),
                new PoseCameraIOPhoton(CAMERA2_NAME, CAMERA2_TRANSFORM3D)
            );

            this.turret = new Turret(
                new TurretIOTalonFX(TurretConstants.CONFIG),
                TurretConstants.ROBOT_TO_TURRET, 
                this.drivetrain.poseEstimator);
        }

        this.drivetrainController = new DrivetrainController(this.drivetrain);

        configureBindings();
        publishAutoNames();
    }

    private void configureBindings() {
        this.controller
            .cross()
            .onTrue(
                new InstantCommand(() -> {
                    this.drivetrain.resetHeading();
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
                    
                    double rotation = -this.controller.getRightX();

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
    }

    private void publishAutoNames() {
        String[] autoNames = {
            // "Wheel Characterization",
            // "Drive Feedforward Characterization",
            // "RotationTuning",
            // "TranslationTuning",
            // "TestPath2026",
            // "BackUpCenter",
            // "BackUpLeft",
            // "OverBump",
            // "VisionTesting2026",
            "Depot Auto",
            "Chaos Right Auto"
        };


        for (String name : autoNames) {
            autoChooser.addOption(name, name);
        }

        autoChooser.setDefaultOption("None", "None");
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand(String name) {
        Command autoCommand = Commands.none();

        switch (name) {
            case "Depot Auto":
                autoCommand = new DepotAuto(this.drivetrain);
                break;
            case "Chaos Right Auto":
                autoCommand = new ChaosRightAuto(this.drivetrain);
                break;
            case "Wheel Characterization":
                autoCommand = DriveCommands.wheelRadiusCharacterization(this.drivetrain);
                break;
            case "Drive Feedforward Characterization":
                autoCommand = DriveCommands.feedforwardCharacterization(this.drivetrain);
                break;
            case "RotationTuning":
                Optional<Trajectory<SwerveSample>> rotationTraj = Choreo.loadTrajectory(
                    "RotationTuning"
                );
                autoCommand = new FollowPath(rotationTraj.get(), this.drivetrain, true);
                break;
            case "TranslationTuning":
                Optional<Trajectory<SwerveSample>> translationTraj = Choreo.loadTrajectory(
                    "TranslationTuning"
                );
                autoCommand = new FollowPath(translationTraj.get(), this.drivetrain, true);
                break;
            case "TestPath2026":
                Optional<Trajectory<SwerveSample>> testPath2026 = Choreo.loadTrajectory(
                    "TestPath2026"
                );
                autoCommand = new FollowPath(testPath2026.get(), this.drivetrain, true);
                break;
            case "BackUpCenter":
                Optional<Trajectory<SwerveSample>> backUpCenter = Choreo.loadTrajectory(
                    "BackUpCenter"
                );
                autoCommand = new FollowPath(backUpCenter.get(), this.drivetrain, true);
                break;
            case "BackUpLeft":
                Optional<Trajectory<SwerveSample>> backUpLeft = Choreo.loadTrajectory(
                    "BackUpLeft"
                );
                autoCommand = new FollowPath(backUpLeft.get(), this.drivetrain, true);
                break;
            case "OverBump":
                Optional<Trajectory<SwerveSample>> overBump = Choreo.loadTrajectory(
                    "OverBump"
                );
                autoCommand = new FollowPath(overBump.get(), this.drivetrain, true);
                break;
            case "VisionTesting2026":
                Optional<Trajectory<SwerveSample>> visionTesting2026 = Choreo.loadTrajectory(
                    "VisionTesting2026"
                );
                autoCommand = new FollowPath(visionTesting2026.get(), this.drivetrain, true); // this path was written on red side
                break;
        }

        autoCommand.setName(name);
        return autoCommand;
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
