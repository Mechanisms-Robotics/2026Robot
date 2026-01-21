/******************************************************************************
 *                                                                            *
 *                 ** MECHIATTO **                                            *
 *                                                                            *
 *  THIS FILE IS FOR MANAGING THE CONSTANTS FOR THE MECHIATTO PRACTICE BOT.   *
 *                                                                            *
 *  DO NOT USE THIS FILE FOR THE COMPETITION BOT.                             *
 *                                                                            *
 *  MAKE SURE TO UPDATE CONSTANTS HERE ONLY IF THEY ARE SPECIFIC TO THE       *
 *  MECHIATTO PRACTICE BOT.                                                   *
 *                                                                            *
 ******************************************************************************/
package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class CONSTANTS {

    //RobotContainer
    public static final int CONTROLLER_PORT = 0;

    // Vision Constants
    public static AprilTagFieldLayout APRILTAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    //Localization
    public static final int GYRO_CAN_ID = 9;

    // Path Following Constants
    public static final double PATH_FOLLOWER_P_X = 2.0;
    public static final double PATH_FOLLOWER_P_Y = 2.0;
    public static final double PATH_FOLLOWER_P_THETA = 4.0;

    public static final double ROBOT_LOOP_PERIOD = 0.02;
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal()
        ? Mode.REAL
        : SIM_MODE;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,
    }

    // NEW DRIVETRAIN CONSTANTS
    public static class DriveConstants {

        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(25.0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(
                StaticFeedforwardSignValue.UseClosedLoopSign
            );
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(0.1)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
            ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
            ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
            DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement STEER_MOTOR_TYPE =
            SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SteerFeedbackType STEER_FEEDBACK_TYPE =
            SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current SLIP_CURRENT = Amps.of(120.0);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
            new TalonFXConfiguration();
        private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
            new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS =
            new CANcoderConfiguration();

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus CAN_BUS = new CANBus("rio");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity SPEED_AT_12_VOLTS =
            MetersPerSecond.of(4.0);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double COUPLE_RATIO = 0.0;

        private static final double DRIVE_GEAR_RATIO = 7.363636363636365;
        private static final double STEER_GEAR_RATIO = 150.0 / 7.0; // ~21.43
        public static final Distance WHEEL_RADIUS = Inches.of(2.0);

        private static final boolean INVERT_LEFT_SIDE = false;
        private static final boolean INVERT_RIGHT_SIDE = true;

        // These are only used for simulation
        private static final MomentOfInertia STEER_INERTIA =
            KilogramSquareMeters.of(0.004);
        private static final MomentOfInertia DRIVE_INERTIA =
            KilogramSquareMeters.of(0.025);
        // Simulated voltage necessary to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
            new SwerveDrivetrainConstants().withCANBusName(CAN_BUS.getName());

        private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > ConstantCreator = new SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        >()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            //.withCouplingGearRatio(COUPLE_RATIO)
            .withWheelRadius(WHEEL_RADIUS)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSlipCurrent(SLIP_CURRENT)
            .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
            .withDriveMotorType(DRIVE_MOTOR_TYPE)
            .withSteerMotorType(STEER_MOTOR_TYPE)
            .withFeedbackSource(STEER_FEEDBACK_TYPE)
            .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
            .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
            .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
            .withSteerInertia(STEER_INERTIA)
            .withDriveInertia(DRIVE_INERTIA)
            .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        // Front Left
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 5;
        private static final int FRONT_LEFT_ENCODER_ID = 1;
        private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(
            0
        );
        private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
        private static final boolean FRONT_LEFT_ENCODER_INVERTED = true;

        private static final Distance FRONT_LEFT_X_POS = Meters.of(0.33);
        private static final Distance FRONT_LEFT_Y_POS = Meters.of(0.23);

        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 6;
        private static final int FRONT_RIGHT_ENCODER_ID = 2;
        private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(
            0
        );
        private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
        private static final boolean FRONT_RIGHT_ENCODER_INVERTED = true;

        private static final Distance FRONT_RIGHT_X_POS = Meters.of(0.33);
        private static final Distance FRONT_RIGHT_Y_POS = Meters.of(-0.23);

        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 8;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 4;
        private static final int BACK_LEFT_ENCODER_ID = 4;
        private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(
            0
        );
        private static final boolean BACK_LELFT_STEER_MOTOR_INVERTED = true;
        private static final boolean BACK_LEFT_ENCODER_INVERTED = true;

        private static final Distance BACK_LEFT_X_POS = Meters.of(-0.33);
        private static final Distance BACK_LEFT_Y_POS = Meters.of(0.23);

        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 3;
        private static final int BACK_RIGHT_ENCODER_ID = 3;
        private static final Angle BACK_RIGHT_ENOCDER_OFFSET = Rotations.of(
            0
        );
        private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
        private static final boolean BACK_RIGHT_ENCODER_INVERTED = true;

        private static final Distance BACK_RIGHT_X_POS = Meters.of(-0.33);
        private static final Distance BACK_RIGHT_Y_POS = Meters.of(-0.23);

        public static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > FRONT_LEFT = ConstantCreator.createModuleConstants(
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_ENCODER_ID,
            FRONT_LEFT_ENCODER_OFFSET,
            FRONT_LEFT_X_POS,
            FRONT_LEFT_Y_POS,
            INVERT_LEFT_SIDE,
            FRONT_LEFT_STEER_MOTOR_INVERTED,
            FRONT_LEFT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > FRONT_RIGHT = ConstantCreator.createModuleConstants(
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_ENCODER_ID,
            FRONT_RIGHT_ENCODER_OFFSET,
            FRONT_RIGHT_X_POS,
            FRONT_RIGHT_Y_POS,
            INVERT_RIGHT_SIDE,
            FRONT_RIGHT_STEER_MOTOR_INVERTED,
            FRONT_RIGHT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > BACK_LEFT = ConstantCreator.createModuleConstants(
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_ENCODER_ID,
            BACK_LEFT_ENCODER_OFFSET,
            BACK_LEFT_X_POS,
            BACK_LEFT_Y_POS,
            INVERT_LEFT_SIDE,
            BACK_LELFT_STEER_MOTOR_INVERTED,
            BACK_LEFT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > BACK_RIGHT = ConstantCreator.createModuleConstants(
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_ENCODER_ID,
            BACK_RIGHT_ENOCDER_OFFSET,
            BACK_RIGHT_X_POS,
            BACK_RIGHT_Y_POS,
            INVERT_RIGHT_SIDE,
            BACK_RIGHT_STEER_MOTOR_INVERTED,
            BACK_RIGHT_ENCODER_INVERTED
        );

        public static final double ODOMETRY_FREQUENCY = new CANBus(
                DriveConstants.DRIVETRAIN_CONSTANTS.CANBusName
            ).isNetworkFD()
            ? 250.0
            : 100.0;

        public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                Math.hypot(
                    DriveConstants.FRONT_LEFT.LocationX,
                    DriveConstants.FRONT_LEFT.LocationY
                ),
                Math.hypot(
                    DriveConstants.FRONT_RIGHT.LocationX,
                    DriveConstants.FRONT_RIGHT.LocationY
                )
            ),
            Math.max(
                Math.hypot(
                    DriveConstants.BACK_LEFT.LocationX,
                    DriveConstants.BACK_LEFT.LocationY
                ),
                Math.hypot(
                    DriveConstants.BACK_RIGHT.LocationX,
                    DriveConstants.BACK_RIGHT.LocationY
                )
            )
        );

        // PathPlanner config constants
        public static final double ROBOT_MASS_KG = 74.088;
        public static final double ROBOT_MOI = 6.883;
        public static final double WHEEL_COF = 1.2;

        public static final double DEADBAND = 0.1;
        public static final double ANGLE_KP = 5.0;
        public static final double ANGLE_KD = 0.4;
        public static final double ANGLE_MAX_VELOCITY = 8.0;
        public static final double ANGLE_MAX_ACCELERATION = 20.0;
        public static final double FF_START_DELAY = 2.0; // Secs
        public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
        public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

        public static final double PATH_PLANNER_KP = 5.0;
        public static final double PATH_PLANNER_KI = 0.0;
        public static final double PATH_PLANNER_KD = 0.0;

        public static final double MM_CRUISE_VELOCITY =
            100.0 / STEER_GEAR_RATIO;
        public static final double MM_ACCELERATION = MM_CRUISE_VELOCITY / 0.100;
        public static final double MM_EXPO_KV = 0.12 * STEER_GEAR_RATIO;
        public static final double MM_EXPO_KA = 0.1;

        public static final int GYRO_CAN_ID = 9;

        public static final double DRIVE_CAN_FRAME_FREQUENCY = 50.0;
        public static final double DRIVE_CAN_FRAME_PERIOD =
            1.0 / DRIVE_CAN_FRAME_FREQUENCY;
        public static final double DRIVE_CAN_FRAME_PERIOD_SEC = 0.01;
        public static final double GRYO_CAN_FRAME_FREQUENCY = 0.01;
    }

    public static class Timeouts {

        public static final double STD_DEBOUNCE_TIME = 0.5; // Secs
        public static final int STD_RETRY_ATTEMPTS = 5;
        public static final double STD_TIMEOUT = 0.1; // Secs
        public static final double STD_TIMEOUT_LONG = 0.25; // Secs
        public static final double SYSID_TIMEOUT = 1.0; // Secs
    }
}
