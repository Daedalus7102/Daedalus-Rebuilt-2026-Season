package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive {
    private final Subsystem m_requirements;

    // Estados de conducción
    public enum SwerveDriveState {
        JOYSTICKS,
        D_PAD,
        IDLE,
        LOCKED,
        ON_THE_FLY,
        AUTO,
    }

    private SwerveDriveState m_state = SwerveDriveState.IDLE;
    private SwerveDriveState m_lastState = null;

    private final SwerveModule m_swerveModule1 = new SwerveModule(
            SwerveConstants.kFrontLeftDriveMotorID,
            SwerveConstants.kFrontLeftTurnMotorID,
            SwerveConstants.kFrontLeftCANcoderID,
            SwerveConstants.kFrontLeftCANcoderOffset,
            SwerveConstants.kFrontLeftDriveInverted);
    private final SwerveModule m_swerveModule2 = new SwerveModule(
            SwerveConstants.kFrontRightDriveMotorID,
            SwerveConstants.kFrontRightTurnMotorID,
            SwerveConstants.kFrontRightCANcoderID,
            SwerveConstants.kFrontRightCANcoderOffset,
            SwerveConstants.kFrontRightDriveInverted);
    private final SwerveModule m_swerveModule3 = new SwerveModule(
            SwerveConstants.kBackLeftDriveMotorID,
            SwerveConstants.kBackLeftTurnMotorID,
            SwerveConstants.kBackLeftCANcoderID,
            SwerveConstants.kBackLeftCANcoderOffset,
            SwerveConstants.kBackLeftDriveInverted);
    private final SwerveModule m_swerveModule4 = new SwerveModule(
            SwerveConstants.kBackRightDriveMotorID,
            SwerveConstants.kBackRightTurnMotorID,
            SwerveConstants.kBackRightCANcoderID,
            SwerveConstants.kBackRightCANcoderOffset,
            SwerveConstants.kBackRightDriveInverted);

    private final SwerveDriveKinematics m_kinematics = SwerveConstants.kKinematics;
    private final Pigeon2 m_gyro = new Pigeon2(SwerveConstants.kPigeonID, SwerveConstants.kCANbus);
    private SwerveDriveOdometry m_odometry;

    private SwerveModulePosition[] m_cachedPositions = new SwerveModulePosition[4];
    private Rotation2d m_cachedRotation = new Rotation2d();
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private boolean m_idleApplied = false;

    private double m_joystickX = 0.0;
    private double m_joystickY = 0.0;
    private double m_joystickOmega = 0.0;
    private double m_dPadXValue = 0.0;
    private double m_dPadYValue = 0.0;

    private final Field2d m_field = new Field2d();
    private DoubleSupplier m_translationX = () -> 0.0, m_translationY = () -> 0.0, m_rotationOmega = () -> 0.0;
    private DoubleSupplier m_padX = () -> 0.0, m_padY = () -> 0.0;
    private double m_deadband = 0.1d;
    private boolean m_fieldRelativeTeleop = true;

    private static final double kLoopPeriodSeconds = 0.02;
    private static final double kDPadDriveScale = 0.3;
    private static final SwerveModuleState[] kLockedStates = {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
    };

    // Constructor
    public SwerveDrive(Subsystem requirements) {
        m_requirements = requirements;
        m_gyro.reset();

        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putBoolean("FieldRelativeTeleop", m_fieldRelativeTeleop);
        SmartDashboard.putNumber("DeadZone", m_deadband);

        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                m_gyro.getRotation2d(),
                readSwerveModulePositions(),
                new Pose2d(0, 0, new Rotation2d())
        );

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getSpeeds,
                    this::driveRobotRelative,
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)
                    ),
                    config,
                    () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Red,
                    m_requirements
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // =======================
    // === CICLO PRINCIPAL ===
    // =======================
    public void periodic() {
        if (m_state != SwerveDriveState.AUTO) {
            sampleDriverInputs();
            runState();
        }

        m_cachedRotation = m_gyro.getRotation2d();
        readSwerveModulePositions();

        m_odometry.update(m_cachedRotation, m_cachedPositions);
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    // ======================
    // === DASHBOARD LOOP ===
    // ======================
    public void updateDashboard() {
        boolean FRT = SmartDashboard.getBoolean("FieldRelativeTeleop", m_fieldRelativeTeleop);
        if (FRT != m_fieldRelativeTeleop) {
            m_fieldRelativeTeleop = FRT;
        }

        double DZ = SmartDashboard.getNumber("DeadZone", m_deadband);
        if (DZ != m_deadband) {
            m_deadband = DZ;
        }

        SmartDashboard.putNumber("Gyro (deg)", m_cachedRotation.getDegrees());
        SmartDashboard.putBoolean("DriverJoystick", isJoystickInputPresent());
        SmartDashboard.putBoolean("DriverDPad", isDPadInputPresent());
    }

    // ======================
    // === STATE MACHINE ===
    // ======================
    private void runState() {
        boolean joystickPresent = isJoystickInputPresent();
        boolean dPadPresent = isDPadInputPresent();

        if (m_state == SwerveDriveState.IDLE) {
            if (joystickPresent) {
                m_state = SwerveDriveState.JOYSTICKS;
            } else if (dPadPresent) {
                m_state = SwerveDriveState.D_PAD;
            }
        } else if (m_state == SwerveDriveState.JOYSTICKS && !joystickPresent) {
            m_state = SwerveDriveState.IDLE;
        } else if (m_state == SwerveDriveState.D_PAD && !dPadPresent) {
            m_state = SwerveDriveState.IDLE;
        }

        if (m_state != m_lastState) {
            SmartDashboard.putString("SwerveDriveState", m_state.name());
            m_lastState = m_state;
        }

        switch (m_state) {
            case JOYSTICKS -> executeTeleopDrive();
            case D_PAD -> executeDPadDrive();
            case LOCKED -> executeLockedDrive();
            case IDLE, ON_THE_FLY, AUTO -> executeIdleDrive();
        }
    }

    public Command setState(SwerveDriveState state) {
        return Commands.runOnce(() -> {
            m_state = state;
            if (state != SwerveDriveState.IDLE) {
                m_idleApplied = false;
            }
        }, m_requirements);
    }

    public void setJoystickSuppliers(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        m_translationX = x;
        m_translationY = y;
        m_rotationOmega = omega;
    }

    public void setDPadSuppliers(DoubleSupplier x, DoubleSupplier y) {
        m_padX = x;
        m_padY = y;
    }

    private void sampleDriverInputs() {
        m_joystickX = MathUtil.applyDeadband(m_translationX.getAsDouble(), m_deadband);
        m_joystickY = MathUtil.applyDeadband(m_translationY.getAsDouble(), m_deadband);
        m_joystickOmega = MathUtil.applyDeadband(m_rotationOmega.getAsDouble(), m_deadband);

        m_dPadXValue = m_padX.getAsDouble();
        m_dPadYValue = m_padY.getAsDouble();
    }

    private boolean isJoystickInputPresent() {
        return m_joystickX != 0.0 || m_joystickY != 0.0 || m_joystickOmega != 0.0;
    }

    private boolean isDPadInputPresent() {
        return m_dPadXValue != 0.0 || m_dPadYValue != 0.0;
    }

    private void executeIdleDrive() {
        if (!m_idleApplied) {
            m_chassisSpeeds = new ChassisSpeeds();
            setSwerveModuleStates(m_chassisSpeeds);
            m_idleApplied = true;
        }
    }

    private void executeLockedDrive() {
        m_idleApplied = false;
        m_swerveModule1.setDesiredState(kLockedStates[0]);
        m_swerveModule2.setDesiredState(kLockedStates[1]);
        m_swerveModule3.setDesiredState(kLockedStates[2]);
        m_swerveModule4.setDesiredState(kLockedStates[3]);
    }

    private void executeTeleopDrive() {
        m_idleApplied = false;
        double x = m_joystickX * SwerveConstants.kDriveMaxSpeed;
        double y = m_joystickY * SwerveConstants.kDriveMaxSpeed;
        double omega = m_joystickOmega * SwerveConstants.kTurnMaxSpeed;
        setSwerveModuleStates(drive(x, y, omega, m_fieldRelativeTeleop, kLoopPeriodSeconds));
    }
    
    private void executeDPadDrive() {
        m_idleApplied = false;
        double x = m_dPadXValue * SwerveConstants.kDriveMaxSpeed * kDPadDriveScale;
        double y = m_dPadYValue * SwerveConstants.kDriveMaxSpeed * kDPadDriveScale;
        setSwerveModuleStates(drive(x, y, 0d, false, kLoopPeriodSeconds));
    }

    private ChassisSpeeds drive(double x, double y, double omega, boolean fieldRelative, double periodSeconds) {
        m_chassisSpeeds = ChassisSpeeds.discretize(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_cachedRotation)
                        : new ChassisSpeeds(x, y, omega),
                periodSeconds);
        return m_chassisSpeeds;
    }

    private SwerveModulePosition[] readSwerveModulePositions() {
        m_cachedPositions[0] = m_swerveModule1.getModulePosition();
        m_cachedPositions[1] = m_swerveModule2.getModulePosition();
        m_cachedPositions[2] = m_swerveModule3.getModulePosition();
        m_cachedPositions[3] = m_swerveModule4.getModulePosition();
        return m_cachedPositions;
    }

    private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kDriveMaxSpeed);

        m_swerveModule1.setDesiredState(states[0]);
        m_swerveModule2.setDesiredState(states[1]);
        m_swerveModule3.setDesiredState(states[2]);
        m_swerveModule4.setDesiredState(states[3]);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(m_cachedRotation, m_cachedPositions, pose);
    }

    public ChassisSpeeds getSpeeds() {
        return m_kinematics.toChassisSpeeds(
                m_swerveModule1.getState(),
                m_swerveModule2.getState(),
                m_swerveModule3.getState(),
                m_swerveModule4.getState());
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        m_idleApplied = false;
        setSwerveModuleStates(chassisSpeeds);
    }

    
}
