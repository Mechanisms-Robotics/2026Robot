// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.function.Supplier;

import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.CONSTANTS.TurretConstants;
import frc.robot.CONSTANTS.VisionConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.commands.autos.Shuttling;
import frc.robot.commands.autos.Beach;
import frc.robot.commands.autos.CenterScore;
import frc.robot.commands.autos.DriveScore;
import frc.robot.commands.autos.LaSiesta;
import frc.robot.commands.autos.MaxScoring;
import frc.robot.commands.autos.MinScoring;
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
    public final SendableChooser<Boolean> enableLimiter = new SendableChooser<>();

    private final Feeder feeder; 

    private final CommandPS4Controller controller = new CommandPS4Controller(
        CONSTANTS.CONTROLLER_PORT
    );
    
    private final HashMap<String, Supplier<Command>> autos = new HashMap<>();

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
                    VisionConstants.CAMERA1_TRANSFORM3D, 
                    drivetrain.poseEstimator
                ),
                new PoseCameraIOSim(
                    "Photon_Camera_Sim2", 
                    VisionConstants.CAMERA2_TRANSFORM3D, 
                    drivetrain.poseEstimator
                )
                );

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
                new PoseCameraIOPhoton(VisionConstants.CAMERA1_NAME, VisionConstants.CAMERA1_TRANSFORM3D),
                new PoseCameraIOPhoton(VisionConstants.CAMERA2_NAME, VisionConstants.CAMERA2_TRANSFORM3D)
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
            this.controller.R1(), // right bumper
            // stow intake button
            this.controller.L1() // left bumper
        );

        
        configureBindings();
        configureTestBindings(); // testing individual mechanisms 
        publishAutoNames();
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        this.enableLimiter.addOption("Disable", false);
        this.enableLimiter.setDefaultOption("Enable", true);
        SmartDashboard.putData("EnableLimiter", this.enableLimiter);
    }

    private void configureBindings() {
        this.controller
            .cross()
            .onTrue(
                new InstantCommand(() -> {
                    this.drivetrain.resetHeading();
                })
            );

        double maxAcceleration = 2.0;
        double maxVelocity = 2.5;
        SlewRateLimiter vxLimiter = new SlewRateLimiter(maxAcceleration);
        SlewRateLimiter vyLimiter = new SlewRateLimiter(maxAcceleration);
        
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

                    ChassisSpeeds limitedSpeeds = new ChassisSpeeds(
                        vxLimiter.calculate(robotOriented.vxMetersPerSecond),
                        vyLimiter.calculate(robotOriented.vyMetersPerSecond),
                        robotOriented.omegaRadiansPerSecond
                    );

                    if (this.controller.R2().getAsBoolean() && this.enableLimiter.getSelected().booleanValue()) {
                        double velocity = Math.hypot(limitedSpeeds.vxMetersPerSecond, limitedSpeeds.vyMetersPerSecond);
                        double slowVelocity = MathUtil.clamp(velocity, -maxVelocity, maxVelocity);
                        double scale = velocity == 0.0 ? 1.0 : slowVelocity / velocity;

                        robotOriented = new ChassisSpeeds(
                            limitedSpeeds.vxMetersPerSecond * scale,
                            limitedSpeeds.vyMetersPerSecond * scale,
                            robotOriented.omegaRadiansPerSecond
                        );
                    }

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
        this.autos.put("Max Scoring Left", () -> new MaxScoring(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, false));
        this.autos.put("Max Scoring Right", () -> new MaxScoring(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, true));
        this.autos.put("Min Scoring Left", () -> new MinScoring(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, false));
        this.autos.put("Min Scoring Right", () -> new MinScoring(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, true));
        this.autos.put("Beach Left", () -> new Beach(this.drivetrain, true));
        this.autos.put("Beach Right", () -> new Beach(this.drivetrain, false));
        this.autos.put("Shuttling Left", () -> new Shuttling(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, false));
        this.autos.put("Shuttling Right", () -> new Shuttling(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, true));
        this.autos.put("LaSiesta Left", () -> new LaSiesta(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, false));
        this.autos.put("LaSiesta Right", () -> new LaSiesta(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, true));
        this.autos.put("Center Score", () -> new CenterScore(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator));
        this.autos.put("Drive Left Score", () -> new DriveScore(this.drivetrain, this.hood, this.flywheel, this.feeder, this.turret, this.intake, this.shotCalculator, new ChassisSpeeds(0.0, 1.0, 0.0), 2.0));

        for (String name : autos.keySet()) {
            autoChooser.addOption(name, name);
        }

        autoChooser.setDefaultOption("None", "None");
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand(String name) {
        Command autoCommand = this.autos.getOrDefault(name, () -> Commands.none()).get();

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
