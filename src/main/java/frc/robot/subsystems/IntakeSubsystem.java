package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the intake roller motor and the Kraken pivot motor.
 *
 * The pivot motor rotates the intake in and out. Two limit switches on the
 * RIO DIO ports define the hard stops:
 *   - Forward limit (deployed)  — stops the pivot when fully extended
 *   - Reverse limit (retracted) — stops the pivot when fully retracted
 *
 * The pivot motor is stopped immediately when either limit switch is triggered.
 * This is checked every loop in periodic() as a safety net in addition to
 * the software commands that stop the motor on command completion.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX    m_rollerMotor;
    private final TalonFX    m_pivotMotor;
    private final DigitalInput m_forwardLimit;
    private final DigitalInput m_reverseLimit;

    // -------------------------------------------------------------------------
    // CAN IDs — non-drivetrain motors are on the RIO CAN bus ("")
    // -------------------------------------------------------------------------
    private static final int    kRollerMotorId        = 9;
    private static final int    kPivotMotorId         = 13; // update to your actual CAN ID
    private static final String kCanBus               = "";

    // -------------------------------------------------------------------------
    // DIO port numbers — update to match your wiring on the RIO
    // -------------------------------------------------------------------------
    private static final int kForwardLimitPort = 0; // deployed limit switch
    private static final int kReverseLimitPort = 1; // retracted limit switch

    // -------------------------------------------------------------------------
    // Motor speeds
    // -------------------------------------------------------------------------
    private static final double kRollerSpeed      =  -.8;
    private static final double kPivotDeploySpeed =  0.3; // positive = deploying
    private static final double kPivotRetractSpeed= -0.3; // negative = retracting

    // -------------------------------------------------------------------------
    // Current limits
    // -------------------------------------------------------------------------
    private static final double kRollerStatorLimit = 80.0;
    private static final double kRollerSupplyLimit = 40.0;
    private static final double kPivotStatorLimit  = 40.0; // lower — pivot is position holding
    private static final double kPivotSupplyLimit  = 30.0;

    public IntakeSubsystem() {
        m_rollerMotor  = new TalonFX(kRollerMotorId, kCanBus);
        m_pivotMotor   = new TalonFX(kPivotMotorId,  kCanBus);
        m_forwardLimit = new DigitalInput(kForwardLimitPort);
        m_reverseLimit = new DigitalInput(kReverseLimitPort);

        configureRoller();
        configurePivot();
    }

    private void configureRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode             = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit       = kRollerStatorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = kRollerSupplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_rollerMotor.getConfigurator().apply(config);
    }

    private void configurePivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Brake mode holds the intake in position when pivot motor is stopped
        config.MotorOutput.NeutralMode             = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit       = kPivotStatorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = kPivotSupplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_pivotMotor.getConfigurator().apply(config);
    }

    // =========================================================================
    // Periodic — limit switch safety enforcement
    // =========================================================================

    @Override
    public void periodic() {
        // Hard stop the pivot if either limit switch is triggered.
        // Limit switches return false when triggered (normally closed circuit).
        // Change to !m_forwardLimit.get() if using normally open switches.
        if (!m_forwardLimit.get() || !m_reverseLimit.get()) {
            stopPivot();
        }

        SmartDashboard.putBoolean("Intake/Deployed",      isDeployed());
        SmartDashboard.putBoolean("Intake/Retracted",     isRetracted());
        SmartDashboard.putNumber("Intake/RollerVelocity", m_rollerMotor.getVelocity().getValueAsDouble());
    }

    // =========================================================================
    // Public getters
    // =========================================================================

    /** Returns true when the forward (deployed) limit switch is triggered. */
    public boolean isDeployed() {
        return !m_forwardLimit.get();
    }

    /** Returns true when the reverse (retracted) limit switch is triggered. */
    public boolean isRetracted() {
        return !m_reverseLimit.get();
    }

    // =========================================================================
    // Private motor helpers
    // =========================================================================

    private void setPivotSpeed(double speed) {
        m_pivotMotor.set(speed);
    }

    private void stopPivot() {
        m_pivotMotor.set(0);
    }

    private void setRollerSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    private void stopRoller() {
        m_rollerMotor.set(0);
    }

    // =========================================================================
    // Commands — pivot
    // =========================================================================

    /**
     * Deploys the intake by running the pivot motor until the forward limit switch
     * triggers, then stops. Safe to call if already deployed.
     */
    public Command deployCommand() {
        return Commands.run(() -> setPivotSpeed(kPivotDeploySpeed), this)
            .until(this::isDeployed)
            .finallyDo(interrupted -> stopPivot());
    }

    /**
     * Retracts the intake by running the pivot motor until the reverse limit switch
     * triggers, then stops. Safe to call if already retracted.
     */
    public Command retractCommand() {
        return Commands.run(() -> setPivotSpeed(kPivotRetractSpeed), this)
            .until(this::isRetracted)
            .finallyDo(interrupted -> stopPivot());
    }

    /**
     * Toggles the intake between deployed and retracted based on current state.
     * Deploys if retracted, retracts if deployed or in between.
     */
    public Command toggleCommand() {
        return Commands.either(
            retractCommand(),
            deployCommand(),
            this::isDeployed
        );
    }

    // =========================================================================
    // Commands — roller
    // =========================================================================

    /** Runs the intake roller at full speed. */
    public Command runCommand() {
        return runOnce(() -> setRollerSpeed(kRollerSpeed));
    }

    /** Stops the intake roller. */
    public Command stopCommand() {
        return runOnce(this::stopRoller);
    }

    /** Reverses the intake roller for unjamming. */
    public Command reverseCommand() {
        return runOnce(() -> setRollerSpeed(-kRollerSpeed));
    }
}