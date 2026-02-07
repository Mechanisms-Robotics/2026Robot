// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.CONSTANTS.VisionConstants;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.CONSTANTS.DriveConstants;

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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.PoseCameraIOPhoton;
import frc.robot.subsystems.vision.PoseCameraIOSim;

public class RobotContainer {

    public final Drivetrain drivetrain;
    private final Vision vision;
    private final DrivetrainController drivetrainController;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
                new PoseCameraIOPhoton(VisionConstants.CAMERA1_NAME, VisionConstants.CAMERA1_TRANSFORM3D)
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

    private void generateAutos() {
        // TODO: We should make sure we don't keep in memory a lot of autos we don't actually need
        // or whatever. This part of the code could be a real memory suck if we're not careful

        autoChooser.setDefaultOption("Wheel Characterization", DriveCommands.wheelRadiusCharacterization(drivetrain));
        autoChooser.addOption("Drive Feedforward Characterization", DriveCommands.feedforwardCharacterization(drivetrain));



        Optional<Trajectory<SwerveSample>> rotationTraj = Choreo.loadTrajectory(
            "RotationTuning"
        );

        Optional<Trajectory<SwerveSample>> translationTraj = Choreo.loadTrajectory(
            "TranslationTuning"
        );
    
        Optional<Trajectory<SwerveSample>> testPath2026 = Choreo.loadTrajectory(
            "TestPath2026"
        );

        Optional<Trajectory<SwerveSample>> acrossTheField = Choreo.loadTrajectory("AcrossTheField");

        autoChooser.addOption("RotationTuning", new FollowPath(rotationTraj.get(), this.drivetrain, true));
        autoChooser.addOption("TranslationTuning", new FollowPath(translationTraj.get(), this.drivetrain, true));
        autoChooser.addOption("TestPath2026", new FollowPath(testPath2026.get(), this.drivetrain, true));
        autoChooser.addOption("AcrossTheField", new FollowPath(acrossTheField.get(), this.drivetrain, true));



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
