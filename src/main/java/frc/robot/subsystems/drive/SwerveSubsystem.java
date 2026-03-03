package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive m_drive;

    public SwerveSubsystem() {
        m_drive = new SwerveDrive(this);
    }

    @Override
    public void periodic() {
        m_drive.periodic();
    }

    public void updateDashboard() {
        m_drive.updateDashboard();
    }

    public Command setState(SwerveDrive.SwerveDriveState state) {
        return m_drive.setState(state);
    }

    public void setJoystickSuppliers(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        m_drive.setJoystickSuppliers(x, y, omega);
    }

    public void setDPadSuppliers(DoubleSupplier x, DoubleSupplier y) {
        m_drive.setDPadSuppliers(x, y);
    }

    public Pose2d getPose() {
        return m_drive.getPose();
    }

    public void resetPose(Pose2d pose) {
        m_drive.resetPose(pose);
    }

    public void zeroGyro() {
        m_drive.zeroGyro();
    }

    /** Vision pose update entry-point (default vision std devs). */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        m_drive.addVisionMeasurement(visionPose, timestampSeconds);
    }

    /** Vision pose update entry-point (custom vision std devs). */
    public void addVisionMeasurement(
            Pose2d visionPose,
            double timestampSeconds,
            Matrix<N3, N1> visionStdDevs) {
        m_drive.addVisionMeasurement(visionPose, timestampSeconds, visionStdDevs);
    }

    /** Optional tuning hook for default vision confidence. */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionStdDevs) {
        m_drive.setVisionMeasurementStdDevs(visionStdDevs);
    }

    public ChassisSpeeds getSpeeds() {
        return m_drive.getSpeeds();
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        m_drive.driveRobotRelative(chassisSpeeds);
    }

    public void setUseFixedOmega(boolean useFixedOmega) {
        m_drive.setUseFixedOmega(useFixedOmega);
    }

    /** Teleop convenience wrapper (angle interpreted with teleop framing rules). */
    public void driveFacingAngle(Rotation2d targetHeading) {
        m_drive.driveFacingAngle(targetHeading);
    }

    /** Teleop convenience wrapper for look-at-point behavior. */
    public void driveFacingPoint(Translation2d targetPoint) {
        m_drive.driveFacingPoint(targetPoint);
    }

    /** Explicit field-heading target API (recommended for autonomous logic). */
    public void aimAtFieldHeading(Rotation2d targetHeading) {
        m_drive.aimAtFieldHeading(targetHeading);
    }

    /** Explicit field-point target API (recommended for autonomous logic). */
    public void aimAtFieldPoint(Translation2d targetPoint) {
        m_drive.aimAtFieldPoint(targetPoint);
    }

    /** Command helper for auto events: set point target and enable aim override. */
    public Command enableAutoAimAtPoint(Translation2d targetPoint) {
        return Commands.runOnce(() -> {
            m_drive.aimAtFieldPoint(targetPoint);
            m_drive.setUseFixedOmega(true);
        });
    }

    /** Command helper for auto events: set heading target and enable aim override. */
    public Command enableAutoAimAtAngle(Rotation2d targetHeading) {
        return Commands.runOnce(() -> {
            m_drive.aimAtFieldHeading(targetHeading);
            m_drive.setUseFixedOmega(true);
        });
    }

    /** Command helper for auto events: disable aim override and return omega control. */
    public Command disableAutoAim() {
        return Commands.runOnce(() -> m_drive.setUseFixedOmega(false));
    }

    /** Immediate (non-command) disable, useful at mode transitions. */
    public void disableAutoAimNow() {
        m_drive.setUseFixedOmega(false);
    }
}
