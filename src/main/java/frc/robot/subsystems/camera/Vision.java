package frc.robot.subsystems.camera;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class Vision {

    private static final AprilTagFieldLayout kFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static class Camera {
        private final String m_name;
        private final PhotonPoseEstimator m_estimator;
        private final PhotonCamera m_camera;
        private final Matrix<N3, N1> m_singleTagDeviation;
        private final Matrix<N3, N1> m_multiTagDeviation;
        private Matrix<N3, N1> m_currentDeviation;

        public Camera(
                String name,
                Transform3d robotToCamOffset,
                Matrix<N3, N1> singleTagDeviation,
                Matrix<N3, N1> multiTagDeviation) {
            m_name = name;
            m_estimator = new PhotonPoseEstimator(kFieldLayout, robotToCamOffset);
            m_camera = new PhotonCamera(name);
            m_singleTagDeviation = singleTagDeviation;
            m_multiTagDeviation = multiTagDeviation;
            m_currentDeviation = singleTagDeviation;
        }

        public Camera(String name, Transform3d robotToCamOffset) {
            this(name, robotToCamOffset, VisionConstants.singleTagDeviation, VisionConstants.multiTagDeviation);
        }

        public List<PhotonPipelineResult> getResults() {
            return m_camera.getAllUnreadResults();
        }

        public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPipelineResult result) {
            Optional<EstimatedRobotPose> pose = m_estimator.estimateCoprocMultiTagPose(result);
            if (pose.isEmpty()) {
                pose = m_estimator.estimateLowestAmbiguityPose(result);
            }
            return pose;
        }

        public void updateDeviation(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> tags) {
            if (estimatedPose.isEmpty()) {
                m_currentDeviation = m_singleTagDeviation;
                return;
            }

            double totalDistance = 0;
            int tagCount = 0;

            for (PhotonTrackedTarget tag : tags) {
                Optional<Pose3d> tagPose = m_estimator.getFieldTags().getTagPose(tag.getFiducialId());
                if (tagPose.isPresent()) {
                    tagCount++;
                    Translation2d estimatedBotTranslation = estimatedPose.get().estimatedPose.toPose2d().getTranslation();
                    Translation2d tagTranslation = tagPose.get().toPose2d().getTranslation();
                    totalDistance += tagTranslation.getDistance(estimatedBotTranslation);
                }
            }

            if (tagCount == 0) {
                m_currentDeviation = m_singleTagDeviation;
                return;
            }

            Matrix<N3, N1> estimatedDeviation = m_singleTagDeviation;
            double averageDistance = totalDistance / tagCount;

            if (tagCount > 1) {
                estimatedDeviation = m_multiTagDeviation;
            }

            if (tagCount == 1 && averageDistance > 4) {
                estimatedDeviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estimatedDeviation = estimatedDeviation.times(1 + (averageDistance * averageDistance / 30));
            }

            m_currentDeviation = estimatedDeviation;
        }

        public Matrix<N3, N1> getCurrentDeviation() {
            return m_currentDeviation;
        }

        public boolean isConnected() {
            return m_camera.isConnected();
        }

        public String getName() {
            return m_name;
        }

        // ── Added for LED system ──
        public boolean hasTargets() {
            for (PhotonPipelineResult result : getResults()) {
                if (result.hasTargets()) return true;
            }
            return false;
        }
    }

    private final Camera[] m_cameras = {
            new Camera(
                    "Camera_Module_Left",
                    new Transform3d(
                            new Translation3d(0.33119, 0.14022, 0.270585),
                            new Rotation3d(0, 0.436332, 0))),
            new Camera(
                    "Camera_Module_Right",
                    new Transform3d(
                            new Translation3d(0.33119, -0.13063, 0.270585),
                            new Rotation3d(0, 0.436332, 0)))
    };

    private final SwerveSubsystem m_swerveSubsystem;

    public Vision(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
    }

    // ── Added for LED system — checks if any camera sees an april tag ──
    public boolean hasAprilTagTarget() {
        for (Camera camera : m_cameras) {
            if (camera.hasTargets()) return true;
        }
        return false;
    }

    public void updatePose() {
        boolean anyPoseThisCycle = false;
        int acceptedCountThisCycle = 0;

        for (Camera camera : m_cameras) {
            String cameraPrefix = camera.getName().contains("Left") ? "Left" : "Right";
            String baseKey = "Vision/" + cameraPrefix + "/";

            List<PhotonPipelineResult> results = camera.getResults();
            SmartDashboard.putBoolean(baseKey + "Connected", camera.isConnected());
            SmartDashboard.putNumber(baseKey + "UnreadResults", results.size());

            boolean hasTargets = false;
            int totalTargets = 0;
            boolean poseValid = false;
            Pose2d latestPose = null;
            double latestTimestamp = -1.0;
            double latestPoseAgeMs = -1.0;

            for (PhotonPipelineResult result : results) {
                hasTargets |= result.hasTargets();
                totalTargets += result.getTargets().size();

                Optional<EstimatedRobotPose> pose = camera.getEstimatedPose(result);
                camera.updateDeviation(pose, result.getTargets());

                if (pose.isPresent()) {
                    EstimatedRobotPose estimatedPose = pose.get();
                    latestPose = estimatedPose.estimatedPose.toPose2d();
                    latestTimestamp = estimatedPose.timestampSeconds;
                    latestPoseAgeMs = (Timer.getFPGATimestamp() - latestTimestamp) * 1000.0;
                    poseValid = true;
                    anyPoseThisCycle = true;
                    acceptedCountThisCycle++;

                    m_swerveSubsystem.addVisionMeasurement(
                            latestPose,
                            latestTimestamp,
                            camera.getCurrentDeviation());
                }
            }

            SmartDashboard.putBoolean(baseKey + "HasTargets", hasTargets);
            SmartDashboard.putNumber(baseKey + "TargetCount", totalTargets);
            SmartDashboard.putBoolean(baseKey + "PoseValid", poseValid);
            SmartDashboard.putNumber(baseKey + "Timestamp", latestTimestamp);
            SmartDashboard.putNumber(baseKey + "PoseAgeMs", latestPoseAgeMs);

            if (latestPose != null) {
                SmartDashboard.putNumber(baseKey + "PoseX", latestPose.getX());
                SmartDashboard.putNumber(baseKey + "PoseY", latestPose.getY());
                SmartDashboard.putNumber(baseKey + "PoseDeg", latestPose.getRotation().getDegrees());
            } else {
                SmartDashboard.putNumber(baseKey + "PoseX", Double.NaN);
                SmartDashboard.putNumber(baseKey + "PoseY", Double.NaN);
                SmartDashboard.putNumber(baseKey + "PoseDeg", Double.NaN);
            }
        }

        SmartDashboard.putBoolean("Vision/AnyPoseThisCycle", anyPoseThisCycle);
        SmartDashboard.putNumber("Vision/AcceptedCountCycle", acceptedCountThisCycle);
    }
}