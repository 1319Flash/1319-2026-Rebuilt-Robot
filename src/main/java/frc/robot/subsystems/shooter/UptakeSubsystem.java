package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the single uptake motor that feeds balls into the flywheel.
 *
 * Runs at full percent output (1.0) with high current limits to maximize
 * available torque. Stator limit of 80A allows the motor to pull hard
 * through resistance without tripping breakers.
 *
 * Non-drivetrain motors are on the RIO CAN bus (kCanBus = "").
 */
public class UptakeSubsystem extends SubsystemBase {
    private final TalonFX m_uptakeMotor;

    // Non-drivetrain motors are on the RIO CAN bus ("")
    private static final int    kUptakeMotorId      = 10;
    private static final String kCanBus             = "";

    private static final double kUptakeSpeed        = -1.0;

    // High stator limit to maximize torque through the uptake mechanism
    // Lower to 60A if the motor runs hot or gearbox shows stress
    private static final double kStatorCurrentLimit = 80.0;
    private static final double kSupplyCurrentLimit = 60.0;

    public UptakeSubsystem() {
        m_uptakeMotor = new TalonFX(kUptakeMotorId, kCanBus);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode holds the ball in place when the uptake stops
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimit       = kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_uptakeMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {}

    // =========================================================================
    // Private helpers
    // =========================================================================

    private void setSpeed(double speed) {
        m_uptakeMotor.set(speed);
    }

    private void stop() {
        m_uptakeMotor.set(0);
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /** Runs the uptake at full speed to feed balls into the flywheel. */
    public Command runCommand() {
        return runOnce(() -> setSpeed(kUptakeSpeed));
    }

    /** Stops the uptake. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /** Reverses the uptake to clear a jam or eject a ball. */
    public Command reverseCommand() {
        return runOnce(() -> setSpeed(-kUptakeSpeed));
    }

    /** Runs the uptake at a custom speed. */
    public Command runAtSpeed(double speed) {
        return runOnce(() -> setSpeed(speed));
    }
}