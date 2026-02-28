package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Pivot limits and presets (relative encoder rotations)
    public static final double kPivotInPosition = 0.0;
    public static final double kPivotOutPosition = 1.2;
    public static final double kPivotMinLimit = 0.0;
    public static final double kPivotMaxLimit = 1.25;

    private static final double kPivotP = 5.0;
    private static final String kPivotTurnsPreferenceKey = "IntakePivotTurns";
    private static final String kPivotAbsZeroPreferenceKey = "IntakePivotAbsZero";

    private final SparkMax m_pivotMotor;
    private final SparkMax m_rollerMotor;
    private final RelativeEncoder m_pivotRelativeEncoder;
    private final AbsoluteEncoder m_pivotAbsoluteEncoder;

    private double m_pivotAbsZeroOffset;

    public IntakeSubsystem() {
        // Pivot motor (intake in/out)
        m_pivotMotor = new SparkMax(IntakeConstants.kRotateMotorID, MotorType.kBrushless);
        m_pivotRelativeEncoder = m_pivotMotor.getAlternateEncoder();
        m_pivotAbsoluteEncoder = m_pivotMotor.getAbsoluteEncoder();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .smartCurrentLimit(35)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false);

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
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(kPivotMinLimit)
            .reverseSoftLimitEnabled(true);

        m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Load persistent absolute "in" zero. If not present, initialize it from current absolute value.
        m_pivotAbsZeroOffset = Preferences.getDouble(kPivotAbsZeroPreferenceKey, Double.NaN);
        if (Double.isNaN(m_pivotAbsZeroOffset)) {
            m_pivotAbsZeroOffset = m_pivotAbsoluteEncoder.getPosition();
            Preferences.setDouble(kPivotAbsZeroPreferenceKey, m_pivotAbsZeroOffset);
        }

        // Reconstruct multi-turn position from absolute reading near last known turns.
        syncRelativeFromAbsolute(Preferences.getDouble(kPivotTurnsPreferenceKey, kPivotInPosition));

        // Roller motor
        m_rollerMotor = new SparkMax(IntakeConstants.kRollerMotorID, MotorType.kBrushless);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .smartCurrentLimit(30)
            .idleMode(SparkMaxConfig.IdleMode.kCoast)
            .inverted(false);

        m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("IntakePosition", m_pivotRelativeEncoder.getPosition());
        SmartDashboard.putNumber("IntakeAbsPosition", m_pivotAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("IntakeAbsZeroOffset", m_pivotAbsZeroOffset);
    }

    public double getPivotPosition() {
        return m_pivotRelativeEncoder.getPosition();
    }

    public void setPivotPosition(double targetPosition) {
        double clampedTarget = MathUtil.clamp(targetPosition, kPivotMinLimit, kPivotMaxLimit);
        m_pivotMotor.getClosedLoopController().setReference(clampedTarget, SparkMax.ControlType.kPosition);
        Preferences.setDouble(kPivotTurnsPreferenceKey, clampedTarget);
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
        // Recalibrate absolute zero so this reference survives reboot/power cycle.
        m_pivotAbsZeroOffset = m_pivotAbsoluteEncoder.getPosition();
        Preferences.setDouble(kPivotAbsZeroPreferenceKey, m_pivotAbsZeroOffset);

        m_pivotRelativeEncoder.setPosition(kPivotInPosition);
        Preferences.setDouble(kPivotTurnsPreferenceKey, kPivotInPosition);
    }

    /** Re-sync relative turns to the absolute reference (best when mechanism is stationary). */
    public void syncRelativeToAbsolute() {
        syncRelativeFromAbsolute(m_pivotRelativeEncoder.getPosition());
    }

    @Override
    public void periodic() {
        Preferences.setDouble(kPivotTurnsPreferenceKey, m_pivotRelativeEncoder.getPosition());
    }

    private void syncRelativeFromAbsolute(double referenceTurns) {
        double estimatedTurns = estimateTurnsFromAbsolute(referenceTurns);
        m_pivotRelativeEncoder.setPosition(estimatedTurns);
        Preferences.setDouble(kPivotTurnsPreferenceKey, estimatedTurns);
    }

    private double estimateTurnsFromAbsolute(double referenceTurns) {
        // Absolute is wrapped [0,1), convert into turns relative to calibrated "in" zero.
        double absWrappedTurns = MathUtil.inputModulus(
            m_pivotAbsoluteEncoder.getPosition() - m_pivotAbsZeroOffset,
            0.0,
            1.0);

        // Pick the nearest full-turn branch to last known turns.
        double candidateTurns = absWrappedTurns + Math.rint(referenceTurns - absWrappedTurns);

        // Keep branch inside legal range.
        while (candidateTurns < kPivotMinLimit) {
            candidateTurns += 1.0;
        }
        while (candidateTurns > kPivotMaxLimit) {
            candidateTurns -= 1.0;
        }

        return MathUtil.clamp(candidateTurns, kPivotMinLimit, kPivotMaxLimit);
    }
}
