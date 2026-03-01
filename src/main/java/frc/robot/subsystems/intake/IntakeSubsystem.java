package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Pivot limits and presets (relative encoder rotations)
    public static final double kPivotInPosition = 0.0;
    public static final double kPivotOutPosition = 1.2;
    public static final double kPivotMinLimit = 0.0;
    public static final double kPivotMaxLimit = 2.5;

    private static final double kPivotP = 5.0;

    private final SparkMax m_pivotMotor;
    private final SparkMax m_rollerMotor;
    private final RelativeEncoder m_pivotRelativeEncoder;

    public IntakeSubsystem() {
        // Pivot motor (intake in/out)
        m_pivotMotor = new SparkMax(IntakeConstants.kRotateMotorID, MotorType.kBrushless);
        m_pivotRelativeEncoder = m_pivotMotor.getAlternateEncoder();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .smartCurrentLimit(35)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(true);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .p(kPivotP)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1);

        pivotConfig.alternateEncoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        pivotConfig.softLimit
            .forwardSoftLimit(kPivotMaxLimit)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(kPivotMinLimit)
            .reverseSoftLimitEnabled(false);

        m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Roller motor
        m_rollerMotor = new SparkMax(IntakeConstants.kRollerMotorID, MotorType.kBrushless);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .smartCurrentLimit(40)
            .idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false);

        m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("IntakePosition", m_pivotRelativeEncoder.getPosition());
    }

    public double getPivotPosition() {
        return m_pivotRelativeEncoder.getPosition();
    }

    public void setPivotPosition(double targetPosition) {
        double clampedTarget = MathUtil.clamp(targetPosition, kPivotMinLimit, kPivotMaxLimit);
        m_pivotMotor.getClosedLoopController().setReference(clampedTarget, SparkMax.ControlType.kPosition);
    }

    public void intakeOut() {
        setPivotPosition(kPivotOutPosition);
    }

    public void intakeIn() {
        setPivotPosition(kPivotInPosition);
    }

    // Optional manual control with software end stops
    public void setPivotManual(double speed) {
        m_pivotMotor.set(speed);
    }

    public void stopPivot() {
        m_pivotMotor.stopMotor();
    }

    // Roller control (simple set as requested)
    public void setRoller(double speed) {
        m_rollerMotor.set(speed);
    }

    public void stopRoller() {
        m_rollerMotor.stopMotor();
    }

    public void stop() {
        stopPivot();
        stopRoller();
    }

    /** Call when the mechanism is physically in to re-zero position tracking. */
    public void zeroPivotAtInPosition() {
        m_pivotRelativeEncoder.setPosition(kPivotInPosition);
    }
}
