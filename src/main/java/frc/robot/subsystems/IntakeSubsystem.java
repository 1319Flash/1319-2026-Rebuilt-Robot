package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the intake motor and pneumatic cylinder used to pick up balls. */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intakeMotor;
    private final DoubleSolenoid m_intakeSolenoid;

    private static final int    kIntakeMotorId      = 9;
    private static final String kCanBus             = "";
    private static final double kIntakeSpeed        = 0.67;

    // Pneumatic channel IDs on the CTRE PCM
    private static final int kSolenoidForwardChannel = 2;
    private static final int kSolenoidReverseChannel = 3;

    // Current limits to protect the motor during jams
    private static final double kStatorCurrentLimit = 40.0;
    private static final double kSupplyCurrentLimit = 30.0;

    // Jam detection — motor is considered jammed if velocity stays below
    // threshold while running for longer than kJamTimeSeconds
    private static final double kJamVelocityThresholdRps = 1.0;  // tune: watch Intake/Velocity while running normally
    private static final double kJamTimeSeconds           = 0.3;  // how long near-zero velocity before declaring a jam
    private static final double kEjectTimeSeconds         = 0.5;  // how long to reverse to clear the jam

    private enum IntakeState { STOPPED, RUNNING, EJECTING }

    private IntakeState m_intakeState = IntakeState.STOPPED;
    private double      m_jamTimer    = 0.0;
    private double      m_ejectTimer  = 0.0;

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

    public boolean getIntakeDoubleSolenoidState() {
    return m_intakeSolenoid.get() == Value.kForward;
  }

    @Override
    public void periodic() {
        double velocity = Math.abs(m_intakeMotor.getVelocity().getValueAsDouble());

        switch (m_intakeState) {
            case RUNNING:
                if (velocity < kJamVelocityThresholdRps) {
                    m_jamTimer += 0.020;
                    if (m_jamTimer >= kJamTimeSeconds) {
                        // Jammed — reverse to eject
                        m_intakeState = IntakeState.EJECTING;
                        m_jamTimer    = 0.0;
                        m_ejectTimer  = 0.0;
                        m_intakeMotor.set(-kIntakeSpeed);
                    }
                } else {
                    m_jamTimer = 0.0; // velocity is healthy, reset timer
                }
                break;

            case EJECTING:
                m_ejectTimer += 0.020;
                if (m_ejectTimer >= kEjectTimeSeconds) {
                    // Ejection done — resume intake
                    m_intakeState = IntakeState.RUNNING;
                    m_ejectTimer  = 0.0;
                    m_jamTimer    = 0.0;
                    m_intakeMotor.set(kIntakeSpeed);
                }
                break;

            case STOPPED:
            default:
                break;
        }

        SmartDashboard.putBoolean("Intake/Extended", getIntakeDoubleSolenoidState());
        SmartDashboard.putBoolean("Intake/Jammed",   m_intakeState == IntakeState.EJECTING);
        SmartDashboard.putString("Intake/State",     m_intakeState.toString());
        SmartDashboard.putNumber("Intake/Velocity",  velocity);
    }

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

    /** Runs the intake at full speed with jam detection active. */
    public Command runCommand() {
        return startEnd(
            () -> {
                m_intakeState = IntakeState.RUNNING;
                m_jamTimer    = 0.0;
                m_intakeMotor.set(kIntakeSpeed);
            },
            () -> {
                m_intakeState = IntakeState.STOPPED;
                m_intakeMotor.set(0);
            }
        );
    }

    /** Stops the intake and cancels any active jam ejection. */
    public Command stopCommand() {
        return runOnce(() -> {
            m_intakeState = IntakeState.STOPPED;
            stop();
        });
    }

    /** Runs the intake at a custom speed. Jam detection is not active. */
    public Command runAtSpeed(double speed) {
        return runOnce(() -> setSpeed(speed));
    }

    /** Manually reverses the intake for unjamming. */
    public Command reverseCommand() {
        return startEnd(
            () -> {
                m_intakeState = IntakeState.EJECTING;
                m_intakeMotor.set(-kIntakeSpeed);
            },
            () -> {
                m_intakeState = IntakeState.STOPPED;
                m_intakeMotor.set(0);
            }
        );
    }
}