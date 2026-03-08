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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class Vision {

    private static final AprilTagFieldLayout kFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static class Camera {
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
    }

    private final Camera[] m_cameras = {
            new Camera(
                    "Camera_Module_Left",
                    new Transform3d(
                            new Translation3d(0.32945, 0.14588, 0.269766),
                            new Rotation3d(0, 0.436332, 0))),
            new Camera(
                    "Camera_Module_Right",
                    new Transform3d(
                            new Translation3d(0.32945, -0.12497, 0.269766),
                            new Rotation3d(0, 0.436332, 0)))
    };

    private final SwerveSubsystem m_swerveSubsystem;

    public Vision(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
    }

    public void updatePose() {
        for (Camera camera : m_cameras) {
            for (PhotonPipelineResult result : camera.getResults()) {
                Optional<EstimatedRobotPose> pose = camera.getEstimatedPose(result);
                camera.updateDeviation(pose, result.getTargets());
                pose.ifPresent(estimatedPose -> m_swerveSubsystem.addVisionMeasurement(
                        estimatedPose.estimatedPose.toPose2d(),
                        estimatedPose.timestampSeconds,
                        camera.getCurrentDeviation()));
            }
        }
    }
}