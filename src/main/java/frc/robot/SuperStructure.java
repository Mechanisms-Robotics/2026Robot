package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.FlywheelConstants;
import frc.robot.ShotCalculator.ShotData;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldUtil;

public class SuperStructure extends SubsystemBase {
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private final PoseEstimator8736 poseEstimator;
    private final ShotCalculator shotCalculator;
    private ShotData shotData = new ShotData(false, null, null, 0);

    private boolean shooting = false;
    private boolean shuttling = false;
    private boolean aiming = false;
    private boolean intaking = false;
    private final BooleanSupplier shootSupplier;
    private final BooleanSupplier intakeSupplier;

    public SuperStructure(Flywheel flywheel, Turret turret, Hood hood, PoseEstimator8736 poseEstimator, BooleanSupplier shootSupplier, BooleanSupplier intakeSupplier) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.hood = hood;
        this.poseEstimator = poseEstimator;
        this.shotCalculator = new ShotCalculator(
            () -> new Pose3d(poseEstimator.getEstimatedPose()).transformBy(new Transform3d()),
            () -> Rotation2d.kZero,
            () -> 0.0
        );

        this.shootSupplier = shootSupplier;
        this.intakeSupplier = intakeSupplier;
    }

    @Override
    public void periodic() {
        Pose2d robotPose = this.poseEstimator.getEstimatedPose();
        shotData = FieldUtil.inAllianceZone(robotPose)
            ? this.shotCalculator.calculateShot(FieldUtil.getHub())
            : this.shotCalculator.calculateShot(FieldUtil.getShuttlePose(robotPose.getY()));

        this.turret.setAngle(shotData.shooterYaw().minus(robotPose.getRotation()));
        
        // MARK: Aim
        if (this.shootSupplier.getAsBoolean()) {
            aiming = true;
            aim();
        } else {
            aiming = false;
            stopAim();
        }

        // MARK: Shoot
        if (this.shootSupplier.getAsBoolean()) {
            if (shotData.aimed()) {
                shooting = true;
                shoot();
            } else {
                shooting = false;
            }
        } else {
            shooting = false;
            stopShoot();
        }

        // MARK: Intake
        if (this.intakeSupplier.getAsBoolean()) {
            intaking = true;
            intake();
        } else if (intaking) {
            intaking = false;
            stowIntake();
        }

        Logger.recordOutput("SuperStructure/aiming", aiming);
        Logger.recordOutput("SuperStructure/aimed", shotData.aimed());
        Logger.recordOutput("SuperStructure/shooting", shooting);
        Logger.recordOutput("SuperStructure/shuttling", shuttling);
        Logger.recordOutput("SuperStructure/intaking", intaking);
    }

    public void aim() {
        this.hood.setAngle(shotData.hoodAngle());
        this.flywheel.setVelocity(shotData.rpm());
    }

    public void stopAim() {
        this.hood.stow();
        this.flywheel.setVelocity(FlywheelConstants.IDLE_RPM);
    }

    public void shoot() {
        /* Spin the indexer and kicker */
    }    

    public void stopShoot() {
        /* Stop spinning the indexer and kicker */
    }

    public void intake() {
        /* deploy intake and spin wheels */
    }

    public void stowIntake() {
        /* stow intake */
    }
}
