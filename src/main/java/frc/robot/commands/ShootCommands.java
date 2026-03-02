package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS.FlywheelConstants;
import frc.robot.PoseEstimator8736;
import frc.robot.ShotCalculator;
import frc.robot.ShotCalculator.ShotData;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;

public class ShootCommands {
    public static class Aim extends Command {
        private final Hood hood;
        private final Flywheel flywheel;
        private final Turret turret;

        private final PoseEstimator8736 poseEstimator;
        private final ShotCalculator shotCalculator;
        private final Supplier<Pose2d> target;
        private ShotData shotData;
        private Pose2d robotPose;

        private static final List<Aim> instances = new ArrayList<>();

        public Aim(Hood hood, Flywheel flywheel, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator, Supplier<Pose2d> target) {
            this.hood = hood;
            this.flywheel = flywheel;

            this.turret = turret;
            this.shotCalculator = shotCalculator;
            this.poseEstimator = poseEstimator;
            this.target = target;
            
            instances.add(this);
            addRequirements(this.hood, this.flywheel);
        }

        public Aim(Hood hood, Flywheel flywheel, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator, Pose2d target) {
            this(hood, flywheel, turret, shotCalculator, poseEstimator, () -> target);
        }

        public boolean isAimed() {
            return Math.abs(this.shotData.rpm() - this.flywheel.getRPM()) < 100
                && Math.abs(this.shotData.hoodAngle().minus(this.hood.getAngle()).getDegrees()) < 3.0
                && Math.abs(this.shotData.shooterYaw().minus(this.turret.getAngle().plus(this.robotPose.getRotation())).getDegrees()) < 3.0;
        }

        public static boolean anyAimed() {
            for (Aim instant : instances) {
                if (instant.isScheduled() && instant.isAimed())
                    return true;
            }
            return false;
        }

        @Override
        public void execute() {
            this.robotPose = this.poseEstimator.getEstimatedPose();
            this.shotData = this.shotCalculator.calculateShot(this.target.get());

            this.hood.setAngle(shotData.hoodAngle());
            this.flywheel.setVelocity(shotData.rpm());
            this.turret.setAngle(shotData.shooterYaw().minus(robotPose.getRotation()));
        }

        @Override
        public void end(boolean interupted) {
            this.hood.stow();
            this.flywheel.setVelocity(FlywheelConstants.IDLE_RPM);
        }
    }

    public static Command aimShuttleCommand(Hood hood, Flywheel flywheel, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Aim aim = new Aim(hood, flywheel, turret, shotCalculator, poseEstimator, () -> FieldUtil.getShuttlePose(poseEstimator.getEstimatedPose().getY()));
        aim.setName("Aim Shuttle");
        return aim;
    }

    public static Command aimHubCommand(Hood hood, Flywheel flywheel, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
        Aim aim = new Aim(hood, flywheel, turret, shotCalculator, poseEstimator, FieldUtil.getHub().toPose2d());
        aim.setName("Aim Hub");
        return aim;
    }

    public static class Shoot extends Command {
        private final Feeder feeder;

        public Shoot(Feeder feeder) {
            this.feeder = feeder;
            addRequirements(this.feeder);
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

    public static class ManualShoot extends Command {
        private final Flywheel flywheel;
        private final Feeder feeder;
        
        private final double rpm;
        
        public ManualShoot(
            Flywheel flywheel,
            Feeder feeder,
            double rpm
        ) {
            this.flywheel = flywheel;
            this.feeder = feeder;

            this.rpm = rpm;
        }

        @Override
        public void initialize() {
            this.flywheel.setVelocity(this.rpm);
            this.feeder.startFeeding();
        }

        @Override
        public void end(boolean interupted) {
            this.feeder.stopFeeding();
        }
    }
}
