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
import frc.robot.RobotContainer;
import frc.robot.subsystems.led.LEDState;

public class IntakeSubsystem extends SubsystemBase {
    // Pivot limits and presets (relative encoder rotations)
    public static final double kPivotInPosition = 0.0;
    public static final double kPivotOutPosition = 19;
    public static final double kPivotMinLimit = 0.0;
    public static final double kPivotMaxLimit = 20;

    private static final double kPivotP = 0.04;

    private final SparkMax m_pivotMotor;
    private final SparkMax m_pivotFollowerMotor;
    private final SparkMax m_rollerMotor;
    private final RelativeEncoder m_pivotEncoder;

    public IntakeSubsystem() {
        // Pivot motor (intake in/out)
        m_pivotMotor = new SparkMax(IntakeConstants.kRotateMotorID, MotorType.kBrushless);
        m_pivotFollowerMotor = new SparkMax(IntakeConstants.kRotateFollowerMotorID, MotorType.kBrushless);
        m_pivotEncoder = m_pivotMotor.getEncoder();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .smartCurrentLimit(35)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(true);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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

        SparkMaxConfig pivotFollowerConfig = new SparkMaxConfig();
        pivotFollowerConfig
            .smartCurrentLimit(35)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .follow(m_pivotMotor, true);

        m_pivotFollowerMotor.configure(
            pivotFollowerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

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
        SmartDashboard.putNumber("IntakePosition", m_pivotEncoder.getPosition());
    }

    public double getPivotPosition() {
        return m_pivotEncoder.getPosition();
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
        m_pivotFollowerMotor.stopMotor();
    }

    // Roller control (simple set as requested)
    public void setRoller(double speed) {
        m_rollerMotor.set(speed);
		RobotContainer.leds.set(LEDState.INTAKE);
    }

    public void stopRoller() {
        m_rollerMotor.stopMotor();
		RobotContainer.leds.set(LEDState.OFF);
    }

    public void stop() {
        stopPivot();
        stopRoller();
    }

    /** Call when the mechanism is physically in to re-zero position tracking. */
    public void zeroPivotAtInPosition() {
        m_pivotEncoder.setPosition(kPivotInPosition);
    }
}
