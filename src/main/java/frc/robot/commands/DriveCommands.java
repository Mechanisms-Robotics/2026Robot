package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveCommands {
    /** Measures the velocity feedforward constants for the drive motors. */
    public static Command feedforwardCharacterization(Drivetrain drivetrain) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }
            ),

            // Allow modules to orient
            Commands.run(() -> drivetrain.runCharacterization(0.0), drivetrain).withTimeout(2.0),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                        double voltage = timer.get() * 0.1;
                        drivetrain.runCharacterization(voltage);
                        velocitySamples.add(drivetrain.getFFCharacterizationVelocity());
                        voltageSamples.add(voltage);
                    },
                drivetrain
            )

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
                        double sumX = 0.0;
                        double sumY = 0.0;
                        double sumXY = 0.0;
                        double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += voltageSamples.get(i);
                            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                        }
                        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                        NumberFormat formatter = new DecimalFormat("#0.00000");
                        System.out.println("********** Drive FF Characterization Results **********");
                        System.out.println("\tkS: " + formatter.format(kS));
                        System.out.println("\tkV: " + formatter.format(kV));
                    }
                )
            );
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drivetrain drivetrain) {
        SlewRateLimiter limiter = new SlewRateLimiter(0.1);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> limiter.reset(0.0)),
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(2.0);
                        drivetrain.setDesiredState(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drivetrain
                )
            ),

            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                        state.positions = drivetrain.getWheelRadiusCharacterizationPositions();
                        state.lastAngle = drivetrain.getGyroRotation();
                        state.gyroDelta = 0.0;
                    }),
                
                // Update gyro delta
                Commands.run(
                    () -> {
                        Rotation2d rotation = drivetrain.getGyroRotation();
                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                        state.lastAngle = rotation;


                        double[] positions = drivetrain.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius =
                            (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                        Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                        Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    }
                )
                
                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drivetrain.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    }
                )
            )
        );
    }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
