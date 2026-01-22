package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {

    // TunerConstants doesn't support separate sim constants, so they are declared locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV =
        1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double TURN_KP = 8.0;
    private static final double TURN_KD = 0.0;
    // private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    // private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(
        DRIVE_KP,
        0,
        DRIVE_KD
    );
    private PIDController turnController = new PIDController(
        TURN_KP,
        0,
        TURN_KD
    );
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor =
            moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));

        this.turnMotor = 
            moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts +
                driveController.calculate(
                    this.moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond)
                );
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(
                this.moduleSimulation.getSteerAbsoluteAngle().in(Radians)
            );
        } else {
            turnController.reset();
        }


        // Update simulation state
        this.driveMotor.requestVoltage(
            Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0))
        );
        this.turnMotor.requestVoltage(
            Volts.of(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0))
        );

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.driveAppliedVolts = this.moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveCurrentAmps = this.moduleSimulation.getDriveMotorStatorCurrent().in(Amps);
        
        inputs.drivePositionRad = 
            this.moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec =
            this.moduleSimulation.getDriveWheelFinalSpeed().in(RevolutionsPerSecond);

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnAppliedVolts = this.moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
        inputs.turnCurrentAmps = this.moduleSimulation.getSteerMotorStatorCurrent().in(Amps);

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(
            this.moduleSimulation.getSteerAbsoluteAngle().in(Rotations)
        );
        inputs.turnPosition = Rotation2d.fromRotations(
            this.moduleSimulation.getSteerAbsoluteAngle().in(Rotations)
        );
        inputs.turnVelocityRadPerSec =
            this.moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);

        inputs.odometryDrivePositionsRad = Arrays.stream(this.moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
        inputs.odometryTurnPositions = this.moduleSimulation.getCachedSteerAbsolutePositions();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts =
            DRIVE_KS * Math.signum(velocityRadPerSec) +
            DRIVE_KV * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
