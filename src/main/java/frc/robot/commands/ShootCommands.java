package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.FlywheelConstants;
import frc.robot.ShotCalculator.ShotData;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;

public class ShootCommands {
    public static class Aim extends Command {
        private final Hood hood;
        private final Flywheel flywheel;
        private final Supplier<ShotData> shotSupplier;

        public Aim(Hood hood, Flywheel flywheel, Supplier<ShotData> shotSupplier) {
            this.hood = hood;
            this.flywheel = flywheel;
            this.shotSupplier = shotSupplier;
        }

        @Override
        public void execute() {
            this.hood.setAngle(this.shotSupplier.get().hoodAngle());
            this.flywheel.setVelocity(this.shotSupplier.get().rpm());
        }

        @Override
        public void end(boolean interupted) {
            this.hood.stow();
            this.flywheel.setVelocity(FlywheelConstants.IDLE_RPM);
        }
    }

    public static class Shoot extends Command {
        private final Feeder feeder;

        public Shoot(Feeder feeder) {
            this.feeder = feeder;
        }

        @Override
        public void initialize() {
            this.feeder.startFeeding();
        }

        @Override
        public void end(boolean interupted) {
            this.feeder.stopFeeding();
        }
    }
}
