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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DepotScoringAuto;
import frc.robot.commands.OutpostScoringAuto;
import frc.robot.commands.ToyAuto;
import frc.robot.commands.UFCLeft;
import frc.robot.commands.UFCRight;
import frc.robot.commands.DepotAndOutpostScoringAuto;
import frc.robot.commands.ManualAutos;
import frc.robot.commands.MaxScoringLeftAuto;
import frc.robot.commands.MaxScoringRightAuto;
import frc.robot.commands.NeutralAndDepotAuto;
import frc.robot.commands.NeutralAndHubBackLeftAuto;
import frc.robot.commands.NeutralAndHubBackRightAuto;
import frc.robot.commands.NeutralAndOutpostAuto;
import frc.robot.commands.NeutralDepotAndOutpostAuto;
import frc.robot.commands.AlbanyLeft;
import frc.robot.commands.AlbanyRight;
import frc.robot.commands.BeachLeftAuto;
import frc.robot.commands.BeachRightAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIORedux;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXRedux;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOSparkMax;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.RollersIOSparkMax;
import frc.robot.subsystems.intake.RollersIOTalonFX;
import frc.robot.subsystems.intake.SlapIOSim;
import frc.robot.subsystems.intake.SlapIOSparkMax;
import frc.robot.subsystems.intake.RollersIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.PoseCameraIOPhoton;
import frc.robot.subsystems.vision.PoseCameraIOSim;

public class RobotContainer {
    public final Drivetrain drivetrain;
    public final Turret turret;
    private final Flywheel flywheel;
    public final Hood hood;
    public final Intake intake;
    
    public final SuperStructure superStructure;
    public final ShotCalculator shotCalculator;
    @SuppressWarnings("unused")
    private final Vision vision;
    private final DrivetrainController drivetrainController;
    
    public final SendableChooser<String> autoChooser = new SendableChooser<>();

    private final Feeder feeder; 

    private final CommandPS4Controller controller = new CommandPS4Controller(
        CONSTANTS.CONTROLLER_PORT
    );

    public RobotContainer() {
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

            this.flywheel = new Flywheel(new FlywheelIOSim());
            this.turret = new Turret(new TurretIOSim());
            this.hood = new Hood(new HoodIOSim());
            this.intake = new Intake(new SlapIOSim(), new RollersIO() {});
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
                // new FeederIO() {},
                // new FeederIO() {}
                new FeederIOTalonFX(
                    CONSTANTS.KICKER_MOTOR_CAN_ID
                ),
                new FeederIOTalonFX(
                    CONSTANTS.SPINDEXER_MOTOR_CAN_ID
                )
            );

            this.intake = new Intake(new SlapIOSparkMax(), new RollersIOTalonFX());
           
            this.vision = new Vision(
                this.drivetrain.poseEstimator,
                new PoseCameraIOPhoton(CAMERA1_NAME, CAMERA1_TRANSFORM3D),
                new PoseCameraIOPhoton(CAMERA2_NAME, CAMERA2_TRANSFORM3D)
            );

            this.flywheel = new Flywheel(new FlywheelIOTalonFX());
            this.hood = new Hood(new HoodIOTalonFX());
            this.turret = new Turret(new TurretIOSparkMax());
        }

        this.drivetrainController = new DrivetrainController(this.drivetrain);
        
        this.shotCalculator = new ShotCalculator(
            () -> new Pose3d(
                this.drivetrain.poseEstimator.getEstimatedPose().transformBy(
                    new Transform2d(
                        TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
                        TurretConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()
                    )
                )
            ),
            () -> ChassisSpeeds.fromRobotRelativeSpeeds(this.drivetrain.getVelocity(), this.drivetrain.getPose().getRotation())
        );
        
        this.superStructure = new SuperStructure(
            this.flywheel,
            this.turret,
            this.hood,
            this.feeder,
            this.intake,
            this.drivetrain.poseEstimator,
            this.shotCalculator,
            // shoot button
            this.controller.R2(), // right trigger
            // intake button
            this.controller.L2(), // left trigger
            // manual mode toggle
            this.controller.button(3),//R1(), // right bumper
            // stow intake button
            this.controller.L1() // left bumper
        );

        configureBindings();
        configureTestBindings(); // testing individual mechanisms 
        publishAutoNames();
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
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

        // you're welcome leif
        new Trigger(() -> !this.superStructure.isWithinSoftLimits())
            .onTrue(
                new InstantCommand(
                    () -> this.controller.setRumble(RumbleType.kBothRumble, 0.5)
                ))
            .onFalse(
                new InstantCommand(
                    () -> this.controller.setRumble(RumbleType.kBothRumble, 0.0)
                ));
    }

    private void configureTestBindings() {
         this.controller
            .square()
            .onTrue(
                new InstantCommand(() -> {
                    this.feeder.stopFeeding();
                })
            );

        this.controller
            .circle()
            .onTrue(
                new InstantCommand(() -> {
                    this.feeder.startFeeding();
                })
            );

        new Trigger(() -> this.controller.getHID().getPOV() == 0)
            .onTrue(
               /*  new InstantCommand(() -> {
                    this.hood.setAngle(this.hood.getAngle().plus(Rotation2d.fromDegrees(CONSTANTS.HOOD_DELTA_DEGREES)));
                })*/

                new InstantCommand(() -> {
                    double newRPM = this.flywheel.getDesiredRPM() + CONSTANTS.FLYWHEEL_DELTA_RPM;
                    this.flywheel.setVelocity(newRPM);
                })
            );

        new Trigger(() -> this.controller.getHID().getPOV() == 180)
            .onTrue(
                new InstantCommand(() -> {
                    //this.hood.setAngle(this.hood.getAngle().minus(Rotation2d.fromDegrees(CONSTANTS.HOOD_DELTA_DEGREES)));

                    double newRPM = this.flywheel.getDesiredRPM() - CONSTANTS.FLYWHEEL_DELTA_RPM;
                    this.flywheel.setVelocity(newRPM);
                })
            );

        new Trigger(() -> this.controller.getHID().getPOV() == 90)
            .onTrue(
                new InstantCommand(() -> {
                    this.hood.changeAngle(Rotation2d.fromDegrees(CONSTANTS.HOOD_DELTA_DEGREES));
                })
            );

        new Trigger(() -> this.controller.getHID().getPOV() == 270)
            .onTrue(
                new InstantCommand(() -> {
                    this.hood.changeAngle(Rotation2d.fromDegrees(-CONSTANTS.HOOD_DELTA_DEGREES));
                })
            );

        // Flywheel: R1 = decrease, R2 = increase
       /* this.controller
            .R1()
            .onTrue(
                new InstantCommand(() -> {
                    double newRPM = this.flywheel.getRPM() - CONSTANTS.FLYWHEEL_DELTA_RPM;
                    this.flywheel.setVelocity(4000);//newRPM);
                })
            );

        this.controller
            .R2()
            .onTrue(
                new InstantCommand(() -> {
                    double newRPM = this.flywheel.getRPM() + CONSTANTS.FLYWHEEL_DELTA_RPM;
                    this.flywheel.setVelocity(4000);
                })
            );*/

        // Kicker: L1 = decrease, L2 = increase
       /*  this.controller
            .L1()
            .onTrue(
                new InstantCommand(() -> {
                    this.feeder.adjustKickerVolts(CONSTANTS.KICKER_DELTA_VOLTS);

                })
            );

        this.controller
            .L2()
            .onTrue(
                new InstantCommand(() -> {
                    this.feeder.adjustKickerVolts(-CONSTANTS.KICKER_DELTA_VOLTS);
                })
            );*/
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
            // "Depot Auto",
            // "Shoot Right Preload",
            // "Shoot Center Preload",
            // "Shoot Left Preload",
            // "Chaos Right Auto",
            // "Chaos Left Auto",
             "Beach Right Auto",
             "Beach Left Auto",
             "Depot Scoring Auto",
            // "Outpost Scoring Auto",
            // "Depot And Outpost Scoring Auto",
            // "Neutral And Outpost Auto",
            // "Neutral And Depot Auto",
            // "Neutral Depot And Outpost Auto",
            // "Neutral And Hub Back Right",
            // "Neutral And Hub Back Left",
            "Max Scoring Auto Right",
            "Max Scoring Auto Left",
            //"Toy Auto",
            "FUC Left",
            "FUC Right",
            "Albany Left",
            "Albany Right"
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
            case "Albany Left":
                autoCommand = new AlbanyLeft(drivetrain, hood, flywheel, feeder, turret, shotCalculator, drivetrain.poseEstimator);
                break;
            case "Albany Right":
                autoCommand = new AlbanyRight(drivetrain, hood, flywheel, feeder, turret, shotCalculator, drivetrain.poseEstimator);
                break;
            case "Shoot Center Preload":
                autoCommand = new ManualAutos.CenterHubBackup(this.drivetrain, this.flywheel, this.feeder);
                break;
            case "Shoot Right Preload":
                autoCommand = new ManualAutos.OutpostBackup(this.drivetrain, this.flywheel, this.feeder);
                break;
            case "Shoot Left Preload":
                autoCommand = new ManualAutos.DepotBackup(this.drivetrain, this.flywheel, this.feeder);
                break;
            case "Depot Scoring Auto":
                autoCommand = new DepotScoringAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Outpost Scoring Auto":
                autoCommand = new OutpostScoringAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Depot And Outpost Scoring Auto":
                autoCommand = new DepotAndOutpostScoringAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Neutral And Outpost Auto":
                autoCommand = new NeutralAndOutpostAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Neutral And Depot Auto":
                autoCommand = new NeutralAndDepotAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Neutral Depot And Outpost Auto":
                autoCommand = new NeutralDepotAndOutpostAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Neutral And Hub Back Right":
                autoCommand = new NeutralAndHubBackRightAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Neutral And Hub Back Left":
                autoCommand = new NeutralAndHubBackLeftAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Max Scoring Auto Right":
                autoCommand = new MaxScoringRightAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "Max Scoring Auto Left":
                autoCommand = new MaxScoringLeftAuto(this.drivetrain, this.hood,this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
            case "FUC Left":
                autoCommand = new UFCLeft(this.drivetrain, this.feeder, this.flywheel);
                break;
            case "FUC Right":
                autoCommand = new UFCRight(this.drivetrain, this.feeder, this.flywheel);
                break;
            case "Beach Right Auto":
                autoCommand = new BeachRightAuto(this.drivetrain);
                break;
            case "Beach Left Auto":
                autoCommand = new BeachLeftAuto(this.drivetrain);
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
            case "Toy Auto":
                autoCommand = new ToyAuto(this.drivetrain, this.hood, this.flywheel, this.feeder, this.intake, this.turret, this.shotCalculator, this.drivetrain.poseEstimator);
                break;
             default:
                 autoCommand = Commands.none();
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
