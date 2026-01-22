// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.CONSTANTS.DriveConstants;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import frc.robot.subsystems.drivetrain.GyroIORedux;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXRedux;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class RobotContainer {

    public final Drivetrain drivetrain;
    public final SwerveDriveSimulation drivetrainSim;
    private final DrivetrainController drivetrainController;

    private final CommandPS4Controller controller = new CommandPS4Controller(
        CONSTANTS.CONTROLLER_PORT
    );

    public RobotContainer() {
        // TODO: Think about where to initialize all of this properly
        if (CONSTANTS.CURRENT_MODE == CONSTANTS.SIM_MODE) {
            this.drivetrainSim = new SwerveDriveSimulation(
                DriveTrainSimulationConfig.Default()
                    .withGyro(COTS.ofGenericGyro())
                    .withRobotMass(Kilograms.of(DriveConstants.ROBOT_MASS_KG))
                    .withBumperSize(Inches.of(30.0), Inches.of(30.0))
                    .withTrackLengthTrackWidth(
                        Meters.of(DriveConstants.TRACK_LENGTH_METERS), Meters.of(DriveConstants.TRACK_WIDTH_METERS)
                    )
                    .withSwerveModule(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getKrakenX60Foc(1),
                        DriveConstants.DRIVE_GEAR_RATIO,
                        DriveConstants.STEER_GEAR_RATIO,
                        DriveConstants.DRIVE_FRICTION_VOLTAGE,
                        DriveConstants.STEER_FRICTION_VOLTAGE,
                        DriveConstants.WHEEL_RADIUS,
                        DriveConstants.STEER_INERTIA,
                        1.5)),
                new Pose2d(3, 3, Rotation2d.kZero));

            SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSim);

            final ModuleIOSim 
                frontLeft = new ModuleIOSim(drivetrainSim.getModules()[0]),
                frontRight = new ModuleIOSim(drivetrainSim.getModules()[1]),
                backLeft = new ModuleIOSim(drivetrainSim.getModules()[2]),
                backRight = new ModuleIOSim(drivetrainSim.getModules()[3]);
            final GyroIOSim gyroIOSim = new GyroIOSim(drivetrainSim.getGyroSimulation());

            this.drivetrain = new Drivetrain(
                gyroIOSim,
                frontLeft,
                frontRight,
                backLeft,
                backRight
            );
        } else {
            this.drivetrainSim = null;
            this.drivetrain = new Drivetrain(
                new GyroIORedux(),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_RIGHT)
            );
        }

        this.drivetrainController = new DrivetrainController(this.drivetrain);

        configureBindings();
        //generateAutos();
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
                        rotation = -this.controller.getRawAxis(4); // Why is sim different then driverstation?
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
    }

    private Command testAuto = null; // this should be replaced when we start developing real autos

    private void generateAutos() {
        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(
            "Test Path"
        );
        this.testAuto = new FollowPath(trajectory.get(), this.drivetrain, true);

        System.out.println("*** Loaded Test Path autonomous ***");
    }

    public Command getAutonomousCommand() {
        return testAuto;
        //return Commands.print("No autonomous command configured");
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
