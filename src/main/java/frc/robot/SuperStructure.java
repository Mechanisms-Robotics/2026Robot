package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShotCalculator.ShotData;
import frc.robot.util.FieldUtil;

public class SuperStructure extends SubsystemBase {
    private final PoseEstimator8736 poseEstimator;
    private final ShotCalculator shotCalculator;

    private boolean shooting = false;
    private boolean shuttling = false;
    private boolean aiming = false;
    private boolean intaking = false;
    private final BooleanSupplier shootSupplier;
    private final BooleanSupplier intakeSupplier;

    public SuperStructure(PoseEstimator8736 poseEstimator, BooleanSupplier shootSupplier, BooleanSupplier intakeSupplier) {
        this.poseEstimator = poseEstimator;
        this.shotCalculator = new ShotCalculator(poseEstimator, () -> 0.0, () -> 0.0, () -> 0.0);
        this.shootSupplier = shootSupplier;
        this.intakeSupplier = intakeSupplier;
    }

    @Override
    public void periodic() {
        boolean aimed = false;

        // MARK: Aim
        if (this.shootSupplier.getAsBoolean()) {
            aiming = true;
            aimed = aim();
        } else {
            aiming = false;
            stopAim();
        }

        // MARK: Shoot
        if (this.shootSupplier.getAsBoolean()) {
            if (aimed) {
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
            stow();
        }
    }

    public boolean aim() {
        Pose2d robotPose = this.poseEstimator.getEstimatedPose();
        if (FieldUtil.inAllianceZone(robotPose)) {
            ShotData shotData = this.shotCalculator.getShot(FieldUtil.getHub());
            return shotData.aimed();
        }
        ShotData shotData = this.shotCalculator.getShot(FieldUtil.getShuttlePose(robotPose.getY()));
        return shotData.aimed();
    }

    public void stopAim() {
        /* Retract the hood and stop powering the flywheel */
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

    public void stow() {
        /* stow intake and stop spinning wheels */
    }
}
