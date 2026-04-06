// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;
  private final SendableChooser<Boolean> resetPoseChooser = new SendableChooser<>();

  public Robot() {
    SignalLogger.enableAutoLogging(false);
    
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

    DriverStation.silenceJoystickConnectionWarning(true);
    this.robotContainer = new RobotContainer();

    resetPoseChooser.setDefaultOption("None", false);
    resetPoseChooser.addOption("All", true);
    SmartDashboard.putData("Reset Pose", resetPoseChooser);
    this.autonomousCommand = Commands.none();
    DriverStation.silenceJoystickConnectionWarning(true);
    // Sets the selected command to None even if elastic already set the auto when the robot turns on
    // Prevents the robot from running an auto that was not intentionally selected after the robot turned on
    SmartDashboard.putString("Auto Chooser/selected", "None");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (resetPoseChooser.getSelected()) {
      robotContainer.drivetrain.resetPose(robotContainer.drivetrain.getPose());
    }
  }

  @Override
  public void robotInit() {
  }
  
  @Override
  public void disabledInit() {}
  
  @Override
  public void disabledPeriodic() {
    String autoName = this.robotContainer.autoChooser.getSelected();
    if (!this.autonomousCommand.getName().equals(autoName)) {
      this.autonomousCommand = this.robotContainer.getAutonomousCommand(autoName);
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (this.autonomousCommand != null) {
      this.autonomousCommand.schedule();
    }
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
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    this.robotContainer.turret.zero();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
