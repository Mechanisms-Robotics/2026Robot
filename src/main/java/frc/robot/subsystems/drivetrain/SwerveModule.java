package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.CONSTANTS.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

import frc.robot.CONSTANTS;

public class SwerveModule {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs =
        new ModuleIOInputsAutoLogged();
    private final String moduleName;

    public SwerveModule(ModuleIO io, String name) {
        this.io = io; // may be real or simulated
        this.moduleName = name;
    }

    public void periodic() {
        // Poll for new hardware inputs, which will be stored in this.inputs
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Drive/Module " + moduleName, this.inputs);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad * CONSTANTS.DriveConstants.WHEEL_RADIUS.in(Meters),
            inputs.turnPosition
        );
    }

    public void setModuleState(SwerveModuleState state) {
        // get the current position and optimize the state
        state.optimize(inputs.turnPosition);

        // calculate a speed scale factor (cosine compensation)
        double scaleFactor = state.angle.minus(inputs.turnPosition).getCos();

        // set the drive velocity (convert m/s to rad/s)
        double driveVelocityRadPerSec =
            (state.speedMetersPerSecond * scaleFactor) / DriveConstants.WHEEL_RADIUS.in(Meters);
        Logger.recordOutput("Module " + this.moduleName + "/Desired Drive Radians Per Second", driveVelocityRadPerSec);
        this.io.setDriveVelocity(driveVelocityRadPerSec);

        // set the turn position
        Logger.recordOutput("Module " + this.moduleName + "/Optimised Angle", state.angle);
        this.io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.kZero);
    }

    public double[] getOdometryTimestamps() {
        return this.inputs.odometryTimestamps;
    }

    public SwerveModulePosition[] getOdometryPositions() {
        int sampleCount = this.inputs.odometryDrivePositionsRad.length;
        SwerveModulePosition[] positions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            positions[i] = new SwerveModulePosition(
                this.inputs.odometryDrivePositionsRad[i] * DriveConstants.WHEEL_RADIUS.in(Meters),
                this.inputs.odometryTurnPositions[i]
            );
        }
        return positions;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }
}
