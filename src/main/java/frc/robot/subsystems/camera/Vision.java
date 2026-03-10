package frc.robot.subsystems.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class Vision {
	public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
			AprilTagFields.k2026RebuiltAndymark);

	public static class Camera {

		private final PhotonPoseEstimator estimator;
		private final PhotonCamera camera;
		private final Matrix<N3, N1> singleTagDeviation;
		private final Matrix<N3, N1> multiTagDeviation;
		private Matrix<N3, N1> currentDeviation;

		public Camera(String name, Transform3d robotToCamOffset, Matrix<N3, N1> singleTagDeviation, Matrix<N3, N1> multiTagDeviation) {
			this.estimator = new PhotonPoseEstimator(fieldLayout, robotToCamOffset);
			this.camera = new PhotonCamera(name);
			this.singleTagDeviation = singleTagDeviation;
			this.multiTagDeviation = multiTagDeviation;
		}

		public Camera(String name, Transform3d robotToCamOffset) {
			this(name, robotToCamOffset, VisionConstants.singleTagDeviation, VisionConstants.multiTagDeviation);
		}

		public List<PhotonPipelineResult> getResults() {
			return camera.getAllUnreadResults();
		}

		public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPipelineResult result) {
			Optional<EstimatedRobotPose> pose = estimator.estimateCoprocMultiTagPose(result);
			if (pose.isEmpty()) {
				pose = estimator.estimateLowestAmbiguityPose(result);
			}
			return pose;
		}

		// code taken from https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
		public void updateDeviation(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> tags) {
			if (estimatedPose.isEmpty())  {
				currentDeviation = singleTagDeviation;
			} else {
				double totalDistance = 0;
				int tagCount = 0;

				for (PhotonTrackedTarget tag : tags) {
					Optional<Pose3d> tagPos = estimator.getFieldTags().getTagPose(tag.getFiducialId());
					if (tagPos.isPresent()) {
						tagCount++;
						Translation2d estimatedBotTranslation = estimatedPose.get().estimatedPose.toPose2d().getTranslation();
						Translation2d tagTranslation = tagPos.get().toPose2d().getTranslation();
						totalDistance += tagTranslation.getDistance(estimatedBotTranslation);
					}
				}

				if (tagCount == 0) {
					currentDeviation = singleTagDeviation;
				} else {
					Matrix<N3, N1> estimatedDeviation = singleTagDeviation;
					double averageDistance = totalDistance / tagCount;

					if (tagCount > 1) estimatedDeviation = multiTagDeviation;

					if (tagCount == 1 && averageDistance > 4)
						estimatedDeviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
					else
						estimatedDeviation = estimatedDeviation.times(1 + (averageDistance * averageDistance / 30));
					currentDeviation = estimatedDeviation;
				}
			}
		}
	}

	public Camera[] cameras = {
			new Camera(
					"Camera_Module_Left",
					new Transform3d(
							new Translation3d(0.32945, 0.14588, 0.269766),
							new Rotation3d(0, 0.436332, 0)
					)
			),
			new Camera(
					"Camera_Module_Right",
					new Transform3d(
							new Translation3d(0.32945, -0.12497, 0.269766),
							new Rotation3d(0, 0.436332, 0)
					)
			)
	};

	private final SwerveSubsystem swerveSubsystem;

	public Vision(SwerveSubsystem swerveSubsystem) {
		this.swerveSubsystem = swerveSubsystem;
	}

	public void updatePose() {
		for (Camera camera : cameras) {
			for (PhotonPipelineResult result : camera.getResults()) {
				Optional<EstimatedRobotPose> pose = camera.getEstimatedPose(result);
				camera.updateDeviation(pose, result.getTargets());
				pose.ifPresent((pose2) -> swerveSubsystem.addVisionMeasurement(pose2, camera.currentDeviation));
			}
		}
	}
}
