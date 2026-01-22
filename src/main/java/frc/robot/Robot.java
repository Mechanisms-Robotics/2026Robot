// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  public Robot() {
    switch (CONSTANTS.CURRENT_MODE) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        //Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
          new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
        );
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    this.robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("Simulation/Robot", this.robotContainer.drivetrainSim.getSimulatedDriveTrainPose());
    Pose3d[] fuelPoses = SimulatedArena.getInstance()
      .getGamePiecesArrayByType("Fuel");
    Logger.recordOutput("Simulation/Fuel", fuelPoses);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    this.autonomousCommand = this.robotContainer.getAutonomousCommand();
    if (this.autonomousCommand != null) {
      this.autonomousCommand.schedule();
    }

    if (Robot.isSimulation())
      resetFieldSim();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (this.autonomousCommand != null) {
      this.autonomousCommand.cancel();
    }
    if (Robot.isSimulation())
      resetFieldSim();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void resetFieldSim() {
    SimulatedArena.getInstance().clearGamePieces();
    final double centerX = 16.541 / 2.0, centerY = 8.09 / 2.0;
    final double width = 1.8, height = 4.45;

    for (double x = centerX - width / 2.0; x < centerX + width / 2.0; x += Units.inchesToMeters(6.0)) {
        for (double y = centerY - height / 2.0; y < centerY + height / 2.0; y += Units.inchesToMeters(6.0)) {
            SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
        }
    }
  }
}
