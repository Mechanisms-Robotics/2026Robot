package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PoseEstimator8736;
import frc.robot.ShotCalculator;
import frc.robot.CONSTANTS.HoodConstants;
import frc.robot.CONSTANTS.TurretConstants;
import frc.robot.ShotCalculator.ShotData;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;

public class ShootCommands {
    public static class Aim extends Command {
        private final Flywheel flywheel;
        private final Turret turret;

        private final PoseEstimator8736 poseEstimator;
        private final ShotCalculator shotCalculator;
        private final Supplier<Boolean> isShuttling;
        private ShotData shotData = new ShotData(Rotation2d.kZero, Rotation2d.fromDegrees(HoodConstants.MIN_DEGREES), 0.0);
        private Pose2d robotPose;

        public Aim(Flywheel flywheel, Turret turret, ShotCalculator shotCalculator, PoseEstimator8736 poseEstimator) {
            this.flywheel = flywheel;
            this.turret = turret;
            this.shotCalculator = shotCalculator;
            this.poseEstimator = poseEstimator;
            this.isShuttling = () -> !FieldUtil.inAllianceZone(this.poseEstimator.getEstimatedPose().getX());
            
            addRequirements(this.flywheel, this.turret);
        }


        public ShotData getShot() {
            return this.shotData;
        }

        @Override
        public void execute() {
            this.robotPose = this.poseEstimator.getEstimatedPose();
            Pose2d target;

            if (this.isShuttling.get()) {
                target = FieldUtil.getShuttlePose(poseEstimator.getEstimatedPose().getY());
            }
            else {
                target = FieldUtil.getHub().toPose2d();
            }
            
            this.shotData = this.shotCalculator.calculateShot(target, this.isShuttling.get());

            this.flywheel.setVelocity(shotData.rpm());
            this.turret.setAngle(shotData.shooterYaw().minus(robotPose.getRotation()));
        }

        @Override
        public void end(boolean interupted) {
            this.turret.setAngle(Rotation2d.fromDegrees(TurretConstants.START_DEGREES));
            this.flywheel.setVoltage(0.0);
        }
    }

    public static class Shoot extends Command {
        private final Feeder feeder;
        private final Hood hood;
        private final Supplier<ShotData> shotData;
        private final Supplier<Boolean> isAimed;

        /**
         * Shoot fuel by spinning spindexer.
         * When the hood is up, the robot cannot go under the trench.
         * Because of this the robot only actuates the hood while shooting.
         * The spindexer only runs (applies open loop voltge) when isAimed is true.
         * 
         * @param feeder feed fuel to flywheel
         * @param hood actuate the hood only while shooting.
         * @param shotData used to get the hood position, get this from ShootCommands.Aim
         * @param isAimed boolean supplier determining if the feeder runs, (hood always actuates)
         */
        public Shoot(Feeder feeder, Hood hood, Supplier<ShotData> shotData, Supplier<Boolean> isAimed) {
            this.feeder = feeder;
            this.hood = hood;
            this.shotData = shotData;
            this.isAimed = isAimed;

            addRequirements(this.feeder, this.hood);
        }

        /**
         * Shoot fuel by spinning spindexer.
         * When the hood is up, the robot cannot go under the trench.
         * Because of this the robot only actuates the hood while shooting.
         * 
         * @param feeder feed fuel to flywheel
         * @param hood actuate the hood only while shooting.
         * @param shotData used to get the hood position, get this from ShootCommands.Aim
         */
        public Shoot(Feeder feeder, Hood hood, Supplier<ShotData> shotData) {
            this(feeder, hood, shotData, () -> true);
        }

        @Override
        public void execute() {
            this.hood.setAngle(this.shotData.get().hoodAngle());
            if (this.isAimed.get()) {
                this.feeder.startFeeding();
            } else {
                this.feeder.stopFeeding();
            }
        }

        @Override
        public void end(boolean interupted) {
            this.feeder.stopFeeding();
            this.hood.stow();
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

            addRequirements(this.flywheel, this.feeder);
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
