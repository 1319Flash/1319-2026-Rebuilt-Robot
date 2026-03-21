package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the intake motor (Falcon 500) and pneumatic deploy cylinder. */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX        m_intakeMotor;
    private final DoubleSolenoid m_intakeSolenoid;

    // Non-drivetrain motors are on the RIO CAN bus ("")
    private static final int    kIntakeMotorId           = 9;
    private static final String kCanBus                  = "";

    private static final double kIntakeSpeed             = -1.0;

    // Pneumatic channel IDs on the REV PH
    private static final int    kSolenoidForwardChannel  = 2;
    private static final int    kSolenoidReverseChannel  = 3;

    // Current limits — protect motor during stall
    private static final double kStatorCurrentLimit      = 40.0;
    private static final double kSupplyCurrentLimit      = 30.0;

    public IntakeSubsystem() {
        m_intakeMotor    = new TalonFX(kIntakeMotorId, kCanBus);
        m_intakeSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            kSolenoidForwardChannel,
            kSolenoidReverseChannel
        );

        // Start retracted so intake is up at robot enable
        m_intakeSolenoid.set(Value.kReverse);

        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimit       = kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_intakeMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake/Extended", isExtended());
        SmartDashboard.putNumber("Intake/Velocity", getVelocity());
    }

    // =========================================================================
    // Private motor helpers
    // =========================================================================

    private void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    private void stop() {
        m_intakeMotor.set(0);
    }

    // =========================================================================
    // Public getters
    // =========================================================================

    /** Returns true if the intake cylinder is currently extended (deployed). */
    public boolean isExtended() {
        return m_intakeSolenoid.get() == Value.kForward;
    }

    public double getVelocity() {
        return m_intakeMotor.getVelocity().getValueAsDouble();
    }

    // =========================================================================
    // Commands — pneumatic
    // =========================================================================

    /** Toggles the intake cylinder between extended and retracted. */
    public Command toggleCommand() {
        return Commands.runOnce(() -> m_intakeSolenoid.toggle(), this);
    }

    /** Extends (deploys) the intake. */
    public Command extendCommand() {
        return Commands.runOnce(() -> m_intakeSolenoid.set(Value.kForward), this);
    }

    /** Retracts the intake. */
    public Command retractCommand() {
        return Commands.runOnce(() -> m_intakeSolenoid.set(Value.kReverse), this);
    }

    // =========================================================================
    // Commands — motor
    // =========================================================================

    /** Runs the intake at full speed. */
    public Command runCommand() {
        return runOnce(() -> setSpeed(kIntakeSpeed));
    }

    /** Stops the intake. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /** Reverses the intake for unjamming. */
    public Command reverseCommand() {
        return runOnce(() -> setSpeed(-kIntakeSpeed));
    }

    /** Runs the intake at a custom speed. */
    public Command runAtSpeed(double speed) {
        return runOnce(() -> setSpeed(speed));
    }
}