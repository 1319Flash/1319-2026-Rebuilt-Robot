package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the dual-motor flywheel shooter.
 *
 * Motor 1 is the primary — runs velocity PID via VelocityVoltage control.
 * Motor 2 follows motor 1 in the opposite direction (opposed) via Phoenix Follower.
 *
 * Velocity model (recalibrate after any shooter mechanical change):
 *   v(d) = kVelocityIntercept + kVelocitySlope * d   (linear least squares fit)
 *   Output clamped to [kMinVelocityRps, kMaxVelocityRps]
 *
 * Current calibration data (LL2 lens to hub AprilTag center):
 *   0.94 m -> 42.5 RPS
 *   1.73 m -> 50.0 RPS
 *   2.67 m -> 60.0 RPS
 *   3.05 m -> 65.0 RPS
 *
 * Non-drivetrain motors run on the RIO CAN bus (kCanBus = "").
 */
public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX          m_shooterMotor1;
    private final TalonFX          m_shooterMotor2;
    private final VelocityVoltage  m_velocityRequest;

    // CAN IDs — non-drivetrain motors are on the RIO bus ("")
    private static final int    kMotor1Id = 11;
    private static final int    kMotor2Id = 12; // defined for documentation; follower references kMotor1Id
    private static final String kCanBus   = "";

    // PID / FF gains — tune feedforward (kS, kV, kA) before kP
    // See Phase 2 tuning document for step-by-step procedure
    private static final double kP = 0.5;  // proportional gain
    private static final double kS = 0.2;  // static friction voltage
    private static final double kV = 0.133; // voltage per RPS
    private static final double kA = 0.01;  // voltage per RPS/s acceleration

    // Current limits — protects motors under stall or during jams
    private static final double kStatorCurrentLimit = 80.0; // amps
    private static final double kSupplyCurrentLimit = 60.0; // amps

    // Velocity model coefficients — update after recalibration
    private static final double kVelocityIntercept = 32.165; // RPS at 0 m
    private static final double kVelocitySlope     = 10.599; // RPS per meter

    // Velocity bounds — clamped output of the model
    private static final double kMinVelocityRps = 30.0;
    private static final double kMaxVelocityRps = 90.0;

    // isAtSpeed() tolerance and auto-only safety timeout
    private static final double kVelocityToleranceRps  = 2.0;  // RPS
    private static final double kAtSpeedTimeoutSeconds = 5.0;  // seconds

    // Current commanded velocity — tracked to prevent isAtSpeed() being true when stopped
    private double m_targetVelocity = 0.0;

    public FlywheelSubsystem() {
        m_shooterMotor1   = new TalonFX(kMotor1Id, kCanBus);
        m_shooterMotor2   = new TalonFX(kMotor2Id, kCanBus);
        m_velocityRequest = new VelocityVoltage(0).withSlot(0);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimit       = kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_shooterMotor1.getConfigurator().apply(config);

        // Motor 2 must have its own config applied BEFORE setControl(Follower)
        // otherwise it runs with factory defaults (no current limits, wrong neutral mode)
        TalonFXConfiguration motor2Config = new TalonFXConfiguration();
        motor2Config.MotorOutput.NeutralMode             = NeutralModeValue.Coast;
        motor2Config.CurrentLimits.StatorCurrentLimit       = kStatorCurrentLimit;
        motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor2Config.CurrentLimits.SupplyCurrentLimit       = kSupplyCurrentLimit;
        motor2Config.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_shooterMotor2.getConfigurator().apply(motor2Config);
        m_shooterMotor2.setControl(new Follower(kMotor1Id, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Velocity",       getCurrentVelocity());
        SmartDashboard.putNumber("Shooter/TargetVelocity", m_targetVelocity);
        SmartDashboard.putBoolean("Shooter/AtSpeed",       isAtSpeed());
    }

    // =========================================================================
    // Getters
    // =========================================================================

    /** Returns the current flywheel velocity in rotations per second. */
    public double getCurrentVelocity() {
        return m_shooterMotor1.getVelocity().getValueAsDouble();
    }

    /**
     * Returns true only when a valid velocity is commanded AND velocity is within tolerance.
     * Returns false when stopped so waitUntilAtSpeed() cannot pass immediately at 0 RPS.
     */
    public boolean isAtSpeed() {
        if (m_targetVelocity < kMinVelocityRps) return false;
        return Math.abs(m_targetVelocity - getCurrentVelocity()) < kVelocityToleranceRps;
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    /**
     * Maps a distance to a flywheel velocity using the linear model.
     * v(d) = kVelocityIntercept + kVelocitySlope * d
     * Clamped to [kMinVelocityRps, kMaxVelocityRps].
     */
    private double calculateVelocityForDistance(double distanceMeters) {
        double velocity = kVelocityIntercept + kVelocitySlope * distanceMeters;
        return Math.max(kMinVelocityRps, Math.min(kMaxVelocityRps, velocity));
    }

    private void setVelocity(double velocityRps) {
        m_targetVelocity = velocityRps;
        m_shooterMotor1.setControl(m_velocityRequest.withVelocity(velocityRps));
    }

    private void setVelocityForDistance(double distanceMeters) {
        setVelocity(calculateVelocityForDistance(distanceMeters));
    }

    private void stop() {
        m_targetVelocity = 0.0;
        m_shooterMotor1.set(0);
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /** Spins up to the model-calculated velocity for the given distance in meters. */
    public Command shootAtDistance(double distanceMeters) {
        return runOnce(() -> setVelocityForDistance(distanceMeters));
    }

    /** Spins up to a specific velocity in RPS. Use for fixed-speed named commands. */
    public Command shootAtVelocity(double velocityRps) {
        return runOnce(() -> setVelocity(velocityRps));
    }

    /** Stops the flywheel and clears the target velocity. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /** Waits until isAtSpeed() returns true. */
    public Command waitUntilAtSpeed() {
        return run(() -> {}).until(this::isAtSpeed);
    }

    /**
     * Waits until isAtSpeed() returns true, but times out in auto so the routine
     * can fail safe instead of hanging forever.
     */
    public Command waitUntilAtSpeedWithTimeout() {
        return waitUntilAtSpeed().withTimeout(kAtSpeedTimeoutSeconds);
    }
}
