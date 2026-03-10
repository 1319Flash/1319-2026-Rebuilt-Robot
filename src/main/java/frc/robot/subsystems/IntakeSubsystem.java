package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the intake motor and pneumatic cylinder used to pick up balls. */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intakeMotor;
    private final DoubleSolenoid m_intakeSolenoid;

    private static final int    kIntakeMotorId      = 9;
    private static final String kCanBus             = "";
    private static final double kIntakeSpeed        = 1.0;

    // Pneumatic channel IDs on the CTRE PCM
    private static final int kSolenoidForwardChannel = 2;
    private static final int kSolenoidReverseChannel = 3;

    // Current limits to protect the motor during jams
    private static final double kStatorCurrentLimit = 40.0;
    private static final double kSupplyCurrentLimit = 30.0;

    public IntakeSubsystem() {
        m_intakeMotor = new TalonFX(kIntakeMotorId, kCanBus);
        m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kSolenoidForwardChannel, kSolenoidReverseChannel);

        // Start retracted
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
    public void periodic() {}

    public void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public void stop() {
        m_intakeMotor.set(0);
    }

    /** Returns true if the intake is currently extended. */
    public boolean isExtended() {
        return m_intakeSolenoid.get() == Value.kForward;
    }

    /** Toggles the intake pneumatic cylinder between extended and retracted. */
    public Command toggleCommand() {
        return Commands.runOnce(() -> m_intakeSolenoid.toggle(), this);
    }

    /** Extends the intake. */
    public Command extendCommand() {
        return Commands.runOnce(() -> m_intakeSolenoid.set(Value.kForward), this);
    }

    /** Retracts the intake. */
    public Command retractCommand() {
        return Commands.runOnce(() -> m_intakeSolenoid.set(Value.kReverse), this);
    }

    /** Runs the intake at full speed to pick up balls. */
    public Command runCommand() {
        return runOnce(() -> setSpeed(kIntakeSpeed));
    }

    /** Stops the intake. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /** Runs the intake at a custom speed. */
    public Command runAtSpeed(double speed) {
        return runOnce(() -> setSpeed(speed));
    }

    /** Reverses the intake for unjamming. */
    public Command reverseCommand() {
        return runOnce(() -> setSpeed(-kIntakeSpeed));
    }
}
