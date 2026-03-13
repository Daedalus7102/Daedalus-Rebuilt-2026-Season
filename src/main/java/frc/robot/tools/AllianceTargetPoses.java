package frc.robot.tools;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Alliance-aware field targets for tower and alliance zones.
 *
 * <p>All coordinates below are placeholders and should be tuned to your final field references.
 */
public final class AllianceTargetPoses {
    private AllianceTargetPoses() {}

    // TODO: Tune these placeholder coordinates.
    private static final Translation2d kBlueTower = new Translation2d(4.626, 4.029);
    private static final Translation2d kRedTower = new Translation2d(11.911, 4.029);

    // TODO: Tune these placeholder coordinates.
    private static final Translation2d kBlueAllianceZoneLeft = new Translation2d(2.5, 6.3);
    private static final Translation2d kBlueAllianceZoneRight = new Translation2d(2.5, 1.9);
    private static final Translation2d kRedAllianceZoneLeft = new Translation2d(14.0, 1.9);
    private static final Translation2d kRedAllianceZoneRight = new Translation2d(14.0, 6.3);

    public static boolean isCurrentAllianceRed() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    /** Returns tower translation based on DriverStation alliance (defaults to Blue if unknown). */
    public static Translation2d getTowerTranslationForCurrentAlliance() {
        return isCurrentAllianceRed() ? kRedTower : kBlueTower;
    }

    /** Returns the tower pose based on DriverStation alliance (zero rotation). */
    public static Pose2d getTowerPoseForCurrentAlliance() {
        return new Pose2d(getTowerTranslationForCurrentAlliance(), Rotation2d.kZero);
    }

    /** Returns alliance-zone left translation based on DriverStation alliance. */
    public static Translation2d getAllianceZoneLeftTranslationForCurrentAlliance() {
        return isCurrentAllianceRed() ? kRedAllianceZoneLeft : kBlueAllianceZoneLeft;
    }

    /** Returns alliance-zone left pose based on DriverStation alliance (zero rotation). */
    public static Pose2d getAllianceZoneLeftPoseForCurrentAlliance() {
        return new Pose2d(getAllianceZoneLeftTranslationForCurrentAlliance(), Rotation2d.kZero);
    }

    /** Returns alliance-zone right translation based on DriverStation alliance. */
    public static Translation2d getAllianceZoneRightTranslationForCurrentAlliance() {
        return isCurrentAllianceRed() ? kRedAllianceZoneRight : kBlueAllianceZoneRight;
    }

    /** Returns alliance-zone right pose based on DriverStation alliance (zero rotation). */
    public static Pose2d getAllianceZoneRightPoseForCurrentAlliance() {
        return new Pose2d(getAllianceZoneRightTranslationForCurrentAlliance(), Rotation2d.kZero);
    }

    /** Returns planar distance from a robot translation to an arbitrary target translation. */
    public static double getDistanceToTarget(Translation2d robotTranslation, Translation2d targetTranslation) {
        Objects.requireNonNull(robotTranslation, "robotTranslation cannot be null");
        Objects.requireNonNull(targetTranslation, "targetTranslation cannot be null");
        return robotTranslation.getDistance(targetTranslation);
    }

    /** Convenience overload when you have robot pose and a translation target. */
    public static double getDistanceToTarget(Pose2d robotPose, Translation2d targetTranslation) {
        Objects.requireNonNull(robotPose, "robotPose cannot be null");
        return getDistanceToTarget(robotPose.getTranslation(), targetTranslation);
    }

    /** Backward-compatible overload using Pose2d target. */
    public static double getDistanceToTarget(Pose2d robotPose, Pose2d targetPose) {
        Objects.requireNonNull(targetPose, "targetPose cannot be null");
        return getDistanceToTarget(robotPose, targetPose.getTranslation());
    }

    /** Returns planar distance from robot pose to alliance tower target. */
    public static double getDistanceToTower(Pose2d robotPose) {
        return getDistanceToTarget(robotPose, getTowerTranslationForCurrentAlliance());
    }

    /** Returns the closest alliance-zone translation (left or right) for the current alliance. */
    public static Translation2d getClosestAllianceZoneTranslation(Pose2d robotPose) {
        Objects.requireNonNull(robotPose, "robotPose cannot be null");

        Translation2d left = getAllianceZoneLeftTranslationForCurrentAlliance();
        Translation2d right = getAllianceZoneRightTranslationForCurrentAlliance();

        double leftDistance = getDistanceToTarget(robotPose, left);
        double rightDistance = getDistanceToTarget(robotPose, right);

        return leftDistance <= rightDistance ? left : right;
    }

    /**
     * Returns the closest alliance-zone pose (left or right) for the current alliance.
     * If equal distance, left is returned.
     */
    public static Pose2d getClosestAllianceZonePose(Pose2d robotPose) {
        return new Pose2d(getClosestAllianceZoneTranslation(robotPose), Rotation2d.kZero);
    }

    /** Returns planar distance from robot pose to the closest alliance-zone pose. */
    public static double getDistanceToClosestAllianceZone(Pose2d robotPose) {
        return getDistanceToTarget(robotPose, getClosestAllianceZoneTranslation(robotPose));
    }
}