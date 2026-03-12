package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;     
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import choreo.trajectory.SwerveSample;
import frc.robot.Constants;
import frc.robot.subsystems.camera.Vision;
import org.photonvision.EstimatedRobotPose;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.util.List;
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Drive modes:
     * <ul>
     *   <li>{@link DriveMode#NORMAL}    — normal field-oriented driving.</li>
     *   <li>{@link DriveMode#AUTO_HUB}  — locks rotation to point at the hub.</li>
     *   <li>{@link DriveMode#AUTO_TEAM} — locks rotation to 0 degrees.</li>
     * </ul>
     */
    public enum DriveMode {
        NORMAL,
        AUTO_HUB,
        AUTO_TEAM
    }

    private final DoubleSupplier joystickX, joystickY, joystickRotation;
    private final DoubleSupplier dPadX, dPadY;
    private DriveMode driveMode = DriveMode.NORMAL;
    private Translation2d hubPos = new Translation2d(0, 0);
    private final double AUTO_HUB_INPUT_SCALE = 0.2;
    private final double DPAD_SCALE = 0.05;
    public double inputMultiplier = 1;
    private boolean scaleInput = false;

    private final Vision vision;
    public final SwerveDrive swerveDrive;

    // ── PID controllers used by followTrajectory()
    // These match the gains from the ChoreoLib "Getting Started" example.
    // Tune kP values to suit your robot; D can stay 0 for most swerve drives.
    private final PIDController xController       = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController       = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5,  0.0, 0.0);

    public SwerveSubsystem(
            DoubleSupplier joystickX,
            DoubleSupplier joystickY,
            DoubleSupplier joystickRotation,
            DoubleSupplier dPadX,
            DoubleSupplier dPadY) {

        this.dPadX = dPadX;
        this.dPadY = dPadY;

        try {
            File configDir = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(configDir)
                    .createSwerveDrive(Constants.SwerveConstants.maxSpeed);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        this.joystickX        = joystickX;
        this.joystickY        = joystickY;
        this.joystickRotation = joystickRotation;
        this.vision           = new Vision(this);

        // Heading is an angle — enable continuous input so the PID doesn't
        // try to spin 350° when a 10° correction is needed.
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            swerveDrive.resetOdometry(new Pose2d(
                new Translation2d(3.53, 7.29),
                new Rotation2d(0)
            ));
        }
    }

    //ChoreoLib trajectory follower
    /**
     * Called by ChoreoLib's AutoFactory every loop cycle during auto.
     * Each {@link SwerveSample} represents the desired robot state at one
     * point in time along the trajectory.
     *
     * <p>The command structure is:
     * <pre>
     *   output = feedforward (from sample's vx/vy/omega)
     *          + feedback    (PID correction for positional error)
     * </pre>
     * viene de la documentacion de choreo
     * @param sample trajectory state interpolated to the current timestamp
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                // X: desired velocity + proportional correction on X position error
                sample.vx + xController.calculate(pose.getX(), sample.x),
                // Y: desired velocity + proportional correction on Y position error
                sample.vy + yController.calculate(pose.getY(), sample.y),
                // Heading: desired angular velocity + correction on heading error
                sample.omega + headingController.calculate(
                        pose.getRotation().getRadians(), sample.heading)
        );

        // driveFieldOriented interprets the speeds in the field frame, not the
        // robot frame, which is what ChoreoLib's samples are expressed in.
        swerveDrive.driveFieldOriented(speeds);
    }

    /** Returns the current robot pose from odometry (+ vision fused). */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /** Resets odometry to the given pose (called at the start of each routine). */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    //Periodic
    @Override
    public void periodic() {
        vision.updatePose();

        Translation2d translation;
        double rotation = getControllerRotation();

        if (usingDPads()) {
            translation = getDPadTranslation();
        } else {
            translation = getControllerTranslation();
        }

        switch (driveMode) {
            case NORMAL    -> drive(translation, rotation, !usingDPads());
            case AUTO_HUB  -> driveAutoHub(translation);
            case AUTO_TEAM -> driveAutoTeam(translation);
        }
    }

    // ── Input scaling ─────────────────────────────────────────────────────────

    public void setScaleInput(boolean scaleInput) {
        this.scaleInput = scaleInput;
    }

    public double getScaleInputValue() {
        return scaleInput ? inputMultiplier : 1;
    }

    public boolean usingDPads() {
        return dPadX.getAsDouble() != 0 || dPadY.getAsDouble() != 0;
    }

    // ── Field visualisation ───────────────────────────────────────────────────

    public void setFieldLine(String objectName, Translation2d startPoint, Translation2d endPoint) {
        Translation2d delta = endPoint.minus(startPoint);
        if (delta.getNorm() < 1e-6) {
            swerveDrive.field.getObject(objectName)
                    .setPoses(new Pose2d(startPoint, Rotation2d.kZero));
            return;
        }

        Rotation2d heading = delta.getAngle();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPoint, heading),
                List.of(),
                new Pose2d(endPoint, heading),
                new TrajectoryConfig(2.0, 2.0));

        swerveDrive.field.getObject(objectName).setTrajectory(trajectory);
    }

    // ── Controller translation / rotation helpers ─────────────────────────────

    private Translation2d getControllerTranslation() {
        return new Translation2d(
                MathUtil.applyDeadband(-joystickY.getAsDouble(), 0.1),
                MathUtil.applyDeadband(-joystickX.getAsDouble(), 0.1)
        ).times(getScaleInputValue() * Constants.SwerveConstants.maxSpeed);
    }

    private Translation2d getDPadTranslation() {
        return new Translation2d(
                dPadX.getAsDouble(),
                dPadY.getAsDouble()
        ).times(getScaleInputValue() * Constants.SwerveConstants.maxSpeed * DPAD_SCALE);
    }

    private double getControllerRotation() {
        return MathUtil.applyDeadband(-joystickRotation.getAsDouble(), 0.1)
                * Constants.SwerveConstants.maxTurnRate * getScaleInputValue();
    }

    // ── Odometry helpers

    public void resetOdometryRotation() {
        swerveDrive.resetOdometry(new Pose2d(
                swerveDrive.getPose().getTranslation(), new Rotation2d(0)));
    }

    // ── Drive mode setters

    public void setHubPos(Translation2d pos) { hubPos = pos; }

    public void setMode(DriveMode mode) { driveMode = mode; }

    public void resetMode() { driveMode = DriveMode.NORMAL; }

    // ── Vision measurement integration
    public void addVisionMeasurement(EstimatedRobotPose pose, Matrix<N3, N1> deviation) {
        swerveDrive.addVisionMeasurement(
                pose.estimatedPose.toPose2d(), pose.timestampSeconds, deviation);
    }

    // ── Internal drive helpers 

    private void driveAutoTeam(Translation2d translation) {
        driveTargetAngle(translation, Math.PI);
    }

    private void driveAutoHub(Translation2d translation) {
        driveTargetAngle(
                translation.times(AUTO_HUB_INPUT_SCALE),
                getRotationToPoint(hubPos));
    }

    private double getRotationToPoint(Translation2d target) {
        Pose2d pose = swerveDrive.getPose();
        Translation2d delta = target.minus(pose.getTranslation());
        return Math.atan2(delta.getY(), delta.getX());
    }

    private void driveTargetAngle(Translation2d translation, double targetAngle) {
        ChassisSpeeds target = swerveDrive.swerveController.getRawTargetSpeeds(
                translation.getX(), translation.getY(),
                targetAngle,
                swerveDrive.getOdometryHeading().getRadians()
        );

        ChassisSpeeds limited = new ChassisSpeeds(
                target.vxMetersPerSecond,
                target.vyMetersPerSecond,
                Math.min(target.omegaRadiansPerSecond, Constants.SwerveConstants.maxTurnRate)
        );

        swerveDrive.driveFieldOriented(limited);
    }

    private void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }
}