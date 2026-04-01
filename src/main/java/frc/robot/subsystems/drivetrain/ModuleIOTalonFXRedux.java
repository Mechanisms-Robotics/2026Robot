package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Fahrenheit;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.reduxrobotics.frames.Frame;
import com.reduxrobotics.frames.FrameData;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.CONSTANTS.Timeouts;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * This class has been modifed to use the Redux Can encoder rather than a ctre encoder.
 */
public class ModuleIOTalonFXRedux implements ModuleIO {

    private final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > constants;

    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final Canandmag cancoder;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(
        0.0
    );
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(
        0.0
    );

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(
        0
    );
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
        new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
        new VelocityTorqueCurrentFOC(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    private final Frame<Double> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(
        Timeouts.STD_DEBOUNCE_TIME,
        Debouncer.DebounceType.kFalling
    );
    private final Debouncer turnConnectedDebounce = new Debouncer(
        Timeouts.STD_DEBOUNCE_TIME,
        Debouncer.DebounceType.kFalling
    );
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(
        Timeouts.STD_DEBOUNCE_TIME,
        Debouncer.DebounceType.kFalling
    );

    public ModuleIOTalonFXRedux(
        SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > constants
    ) {
        this.constants = constants;
        this.driveTalon = new TalonFX(
            constants.DriveMotorId,
            DriveConstants.DRIVETRAIN_CONSTANTS.CANBusName
        );
        this.turnTalon = new TalonFX(
            constants.SteerMotorId,
            DriveConstants.DRIVETRAIN_CONSTANTS.CANBusName
        );
        this.cancoder = new Canandmag(constants.EncoderId);

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio =
            constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () ->
            this.driveTalon.getConfigurator().apply(driveConfig, 0.25)
        );
        tryUntilOk(5, () -> this.driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;

        // Use just the steering motor encoder, as CTRE does not support uing fused encoders
        // with third party encoders
        turnConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RotorSensor;

        // Update to use the gear ration of the steer motor rather than the steer ration
        // after the can encoder
       // turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.Feedback.SensorToMechanismRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity =
            DriveConstants.MM_CRUISE_VELOCITY;
        turnConfig.MotionMagic.MotionMagicAcceleration =
            DriveConstants.MM_ACCELERATION;
        turnConfig.MotionMagic.MotionMagicExpo_kV = DriveConstants.MM_EXPO_KV;
        turnConfig.MotionMagic.MotionMagicExpo_kA = DriveConstants.MM_EXPO_KA;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true; // ALWAYS TRUE FOR STEER MOTOR
        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () ->
            this.turnTalon.getConfigurator().apply(turnConfig, 0.25)
        );

        // Configure CANCoder
        // We use the zero button instead of the code configuration
        //cancoder.setAbsPosition(
        //    constants.EncoderOffset,
        //    CONSTANTS.Timeouts.STD_TIMEOUT_LONG
        //);
        CanandmagSettings settings = new CanandmagSettings();
        settings.setInvertDirection(constants.EncoderInverted);
        settings.setPositionFramePeriod(
            DriveConstants.DRIVE_CAN_FRAME_PERIOD_SEC
        );
        cancoder.setSettings(settings, CONSTANTS.Timeouts.STD_TIMEOUT_LONG);

        // Create timestamp queue
        this.timestampQueue =
            PhoenixOdometryThread.getInstance().makeTimestampQueue();

        // Create drive status signals
        this.drivePosition = this.driveTalon.getPosition();
        this.drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(
                this.drivePosition.clone()
            );
        this.driveVelocity = this.driveTalon.getVelocity();
        this.driveAppliedVolts = this.driveTalon.getMotorVoltage();
        this.driveCurrent = this.driveTalon.getSupplyCurrent();

        // Create turn status signals
        this.turnAbsolutePosition = this.cancoder.getAbsPositionFrame();
        this.turnPosition = this.turnTalon.getPosition();
        this.turnPositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(
                this.turnPosition.clone()
            );
        this.turnVelocity = this.turnTalon.getVelocity();
        this.turnAppliedVolts = this.turnTalon.getMotorVoltage();
        this.turnCurrent = this.turnTalon.getSupplyCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.ODOMETRY_FREQUENCY,
            this.drivePosition,
            this.turnPosition
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.DRIVE_CAN_FRAME_FREQUENCY,
            this.driveVelocity,
            this.driveAppliedVolts,
            this.driveCurrent,
            this.turnVelocity,
            this.turnAppliedVolts,
            this.turnCurrent
        );
        ParentDevice.optimizeBusUtilizationForAll(
            this.driveTalon,
            this.turnTalon
        );

        // Reset the turn motor position based on the can encoder
        // Cast is safe as defined in the reducx docs
        FrameData<?>[] data = Frame.waitForFrames(
                Timeouts.STD_TIMEOUT_LONG,
                this.turnAbsolutePosition
        );
        tryUntilOk(10, () ->
            turnTalon.setPosition(
                this.cancoder.getAbsPosition(),
                CONSTANTS.Timeouts.STD_TIMEOUT_LONG
            )
        );
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(
            this.drivePosition,
            this.driveVelocity,
            this.driveAppliedVolts,
            this.driveCurrent
        );
        var turnStatus = BaseStatusSignal.refreshAll(
            this.turnPosition,
            this.turnVelocity,
            this.turnAppliedVolts,
            this.turnCurrent
        );
        // var turnEncoderStatus = Frame.waitForFrames(
        //     Timeouts.STD_TIMEOUT,
        //     this.turnAbsolutePosition
        // );

        // Update drive inputs
        inputs.driveConnected = this.driveConnectedDebounce.calculate(
            driveStatus.isOK()
        );
        inputs.drivePositionRad = Units.rotationsToRadians(
            this.drivePosition.getValueAsDouble()
        );
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(
            this.driveVelocity.getValueAsDouble()
        );
        inputs.driveAppliedVolts = this.driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = this.driveCurrent.getValueAsDouble();
        inputs.driveTempFahrenheit = this.driveTalon.getDeviceTemp().getValue().in(Fahrenheit);

        // Update turn inputs
        inputs.turnConnected = this.turnConnectedDebounce.calculate(
            turnStatus.isOK()
        );
        inputs.turnEncoderConnected =
            this.turnEncoderConnectedDebounce.calculate(
                true
                //turnEncoderStatus != null
            );
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(
            this.cancoder.getAbsPosition()
        );
        inputs.turnPosition = Rotation2d.fromRotations(
            this.turnPosition.getValueAsDouble()
        );
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(
            this.turnVelocity.getValueAsDouble()
        );
        inputs.turnAppliedVolts = this.turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = this.turnCurrent.getValueAsDouble();
        inputs.turnTempFahrenheit = this.turnTalon.getDeviceTemp().getValue().in(Fahrenheit);

        // Update odometry inputs
        inputs.odometryTimestamps = this.timestampQueue.stream()
            .mapToDouble((Double value) -> value)
            .toArray();
        inputs.odometryDrivePositionsRad = this.drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
        inputs.odometryTurnPositions = this.turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
        this.timestampQueue.clear();
        this.drivePositionQueue.clear();
        this.turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        this.driveTalon.setControl(
            switch (this.constants.DriveMotorClosedLoopOutput) {
                case Voltage -> this.voltageRequest.withOutput(output);
                case TorqueCurrentFOC -> this.torqueCurrentRequest.withOutput(
                    output
                );
            }
        );
    }

    @Override
    public void setTurnOpenLoop(double output) {
        this.turnTalon.setControl(
            switch (this.constants.SteerMotorClosedLoopOutput) {
                case Voltage -> this.voltageRequest.withOutput(output);
                case TorqueCurrentFOC -> this.torqueCurrentRequest.withOutput(
                    output
                );
            }
        );
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        this.driveTalon.setControl(
            switch (this.constants.DriveMotorClosedLoopOutput) {
                case Voltage -> this.velocityVoltageRequest.withVelocity(
                    velocityRotPerSec
                );
                case TorqueCurrentFOC -> this.velocityTorqueCurrentRequest.withVelocity(
                    velocityRotPerSec
                );
            }
        );
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        this.turnTalon.setControl(
            switch (this.constants.SteerMotorClosedLoopOutput) {
                case Voltage -> this.positionVoltageRequest.withPosition(
                    rotation.getRotations()
                );
                case TorqueCurrentFOC -> this.positionTorqueCurrentRequest.withPosition(
                    rotation.getRotations()
                );
            }
        );
    }
}
