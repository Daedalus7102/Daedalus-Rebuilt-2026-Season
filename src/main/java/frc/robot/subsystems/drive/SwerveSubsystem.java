package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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

    public ChassisSpeeds getSpeeds() {
        return m_drive.getSpeeds();
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        m_drive.driveRobotRelative(chassisSpeeds);
    }

    public void setUseFixedOmega(boolean useFixedOmega) {
        m_drive.setUseFixedOmega(useFixedOmega);
    }

    public void driveFacingAngle(Rotation2d targetHeading) {
        m_drive.driveFacingAngle(targetHeading);
    }

    public void driveFacingPoint(Translation2d targetPoint) {
        m_drive.driveFacingPoint(targetPoint);
    }
}
