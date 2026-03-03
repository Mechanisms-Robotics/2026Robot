package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Slam;

public class IntakeCommands {
    /**
     * Deploy the intake and spin the rollers until the command is interupted.
     * When this command finishes, it stops the motors and retracts the intake.
     */
    public static Command intake(Slam intake) {
        return Commands.runEnd(
            () -> intake.deploy(),
            () -> intake.retract(),
            intake
        );
    }
}
