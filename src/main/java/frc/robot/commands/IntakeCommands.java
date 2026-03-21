package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    /**
     * Deploy the intake and spin the rollers until the command is interupted.
     * When this command finishes, it retracts the intake to the feed position.
     */
    public static Command intake(Intake intake) {
        return Commands.runEnd(
            () -> intake.deploy(),
            () -> intake.feed(),
            intake
        );
    }

    /** Bring the intake arms up to the stow position */
    public static Command stow(Intake intake) {
        return Commands.runOnce(
            intake::stow, 
            intake
        );
    }

    /** Bring the intake arms down to the position to intake balls */
    public static Command deploy(Intake intake) {
        return Commands.runOnce(
            intake::deploy,
            intake
        );
    }

  
    /** Bring the intake arms up a bit for the feeding position*/
    public static Command feed(Intake intake) {
        return Commands.runOnce(
            intake::feed,
            intake
        );
    }
}
