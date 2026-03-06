package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    // Module
    private SwerveModuleState m_moduleState = new SwerveModuleState();
    // Motors
    private final SparkMax m_driveMotor;
    private final SparkMaxConfig m_driveMotorConfig;
    private final RelativeEncoder m_driveRelativeEncoder;

    private final SparkMax m_turnMotor;
    private final SparkMaxConfig m_turnMotorConfig;
    private final RelativeEncoder m_turnRelativeEncoder;
    // Absolute Encoder
    private final CANcoder m_turnEncoder;
    private final CANcoderConfiguration m_turnEncoderConfig;
    private final double m_offset;

    // Class Constructor
    public SwerveModule(int driveMotorID, int turnMotorID, int CANcoderID , double turnEncoderOffset, boolean isInverted) {
        // Drive Motor
        m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        m_driveRelativeEncoder = m_driveMotor.getEncoder();
        m_driveMotorConfig = new SparkMaxConfig();
        m_driveMotorConfig
            .smartCurrentLimit(SwerveConstants.kDriveCurrentLimitA)
            .idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(isInverted)
            .voltageCompensation(SwerveConstants.kDriveVoltageComp)
            .openLoopRampRate(SwerveConstants.kDriveOpenLoopRamp)
            .closedLoopRampRate(SwerveConstants.kDriveClosedLoopRamp);
        m_driveMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(SwerveConstants.kDriveP)
            .i(SwerveConstants.kDriveI)
            .d(SwerveConstants.kDriveD)
            .outputRange(-1, 1);
        m_driveMotorConfig.closedLoop.feedForward
            .kV(SwerveConstants.kDriveFF);
        m_driveMotorConfig.encoder
            .positionConversionFactor(SwerveConstants.kDriveGearRatio)
            .velocityConversionFactor(SwerveConstants.kDriveVelocityFactor);

        // Turn Motor
        m_turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
        m_turnRelativeEncoder = m_turnMotor.getEncoder();
        m_turnMotorConfig = new SparkMaxConfig();
        m_turnMotorConfig
            .smartCurrentLimit(SwerveConstants.kTurnCurrentLimitA)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(SwerveConstants.kTurnVoltageComp)
            .openLoopRampRate(SwerveConstants.kTurnOpenLoopRamp)
            .closedLoopRampRate(SwerveConstants.kTurnClosedLoopRamp);
        m_turnMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(SwerveConstants.kTurnP)
            .i(SwerveConstants.kTurnI)
            .d(SwerveConstants.kTurnD)
            .outputRange(-1, 1);
        m_turnMotorConfig.encoder
            .positionConversionFactor(SwerveConstants.kTurnPositionFactor)
            .velocityConversionFactor(SwerveConstants.kTurnVelocityFactor);

        // Absolute Encoder
        m_offset = turnEncoderOffset;
        m_turnEncoder = new CANcoder(CANcoderID, new CANBus(SwerveConstants.kCANbus));
        m_turnEncoderConfig = new CANcoderConfiguration();
        m_turnEncoderConfig
            .MagnetSensor.MagnetOffset = m_offset;
        m_turnEncoderConfig
            .MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // Apply Configs
        m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_turnMotor.configure(m_turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_turnEncoder.getConfigurator().apply(m_turnEncoderConfig);

        syncAzimuthToAbsolute();
    }

    // Brake Configuration
    public void motorsToBrake() {
        m_driveMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        m_turnMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_turnMotor.configure(m_turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Coast Configuration
    public void motorsToCoast() {
        m_driveMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        m_turnMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_turnMotor.configure(m_turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Stop Motors
    public void stop(){
        m_driveMotor.stopMotor();
        m_turnMotor.stopMotor();
    }

    // Set to Desired State
    public void setDesiredState(SwerveModuleState desired) {
        double currentAzimuthDeg = m_turnRelativeEncoder.getPosition();
        var current = Rotation2d.fromDegrees(currentAzimuthDeg);
        SwerveModuleState optimizedDesired = new SwerveModuleState(
            desired.speedMetersPerSecond,
            desired.angle);
        optimizedDesired.optimize(current);
        m_moduleState = new SwerveModuleState(
            optimizedDesired.speedMetersPerSecond,
            optimizedDesired.angle);

        m_driveMotor.getClosedLoopController().setSetpoint(
            optimizedDesired.speedMetersPerSecond,
            SparkMax.ControlType.kVelocity);

        // Keep the fast control loop on the relative encoder to avoid unnecessary CANcoder reads.
        // CANcoder absolute is used for initial alignment in syncAzimuthToAbsolute().
        double deltaDeg = MathUtil.inputModulus(
            optimizedDesired.angle.getDegrees() - currentAzimuthDeg,
            -180.0,
            180.0);
        double setPoint = currentAzimuthDeg + deltaDeg;

        m_turnMotor.getClosedLoopController().setSetpoint(
            setPoint,
            SparkMax.ControlType.kPosition);
    }

    // Coordinates Encoder Position
    public final void syncAzimuthToAbsolute() {
        // Phoenix 6: getAbsolutePosition() returns rotations in [0,1)
        double absRot = m_turnEncoder.getAbsolutePosition().getValueAsDouble();
        double absDeg = absRot * 360;

        m_turnRelativeEncoder.setPosition(absDeg);
    }

    // Get Module Position (Used for Odometry)
    public SwerveModulePosition getModulePosition() {
        Rotation2d rotation2d = Rotation2d.fromDegrees(m_turnRelativeEncoder.getPosition());
        double position = m_driveRelativeEncoder.getPosition() * SwerveConstants.kWheelCircumference;
        return new SwerveModulePosition(position, rotation2d);
    }

    // Reset Drive Encoder Position (Used for Odometry)
    public void resetModulePosition() {
        m_driveRelativeEncoder.setPosition(0);
    }

    // Get Current State
    public SwerveModuleState getState(){
        return m_moduleState;
    }

}
