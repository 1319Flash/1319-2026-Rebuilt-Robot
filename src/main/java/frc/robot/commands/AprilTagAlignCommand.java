package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightSubsystem;

/**
 * Rotates the robot to align with an AprilTag using the Limelight.
 * The driver retains full control of translation. Only rotation is automated.
 */
public class AprilTagAlignCommand extends Command {
    private static final double kJoystickDeadband = 0.1;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final LimelightSubsystem      m_limelight;
    private final DoubleSupplier          m_forwardSupplier;
    private final DoubleSupplier          m_strafeSupplier;
    private final DoubleSupplier          m_speedMultiplierSupplier;
    private final double                  m_maxSpeed;
    private final double                  m_maxAngularRate;

    private final SwerveRequest.RobotCentric m_robotCentricRequest;

    public AprilTagAlignCommand(
        CommandSwerveDrivetrain drivetrain,
        LimelightSubsystem limelight,
        DoubleSupplier forwardSupplier,
        DoubleSupplier strafeSupplier,
        double maxSpeed,
        double maxAngularRate,
        DoubleSupplier speedMultiplierSupplier
    ) {
        m_drivetrain              = drivetrain;
        m_limelight               = limelight;
        m_forwardSupplier         = forwardSupplier;
        m_strafeSupplier          = strafeSupplier;
        m_maxSpeed                = maxSpeed;
        m_maxAngularRate          = maxAngularRate;
        m_speedMultiplierSupplier = speedMultiplierSupplier;

        m_robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        m_limelight.setLEDsOn();
    }

    @Override
    public void execute() {
        double rotationalRate  = m_limelight.calculateAimVelocity(m_maxAngularRate);
        double speedMultiplier = m_speedMultiplierSupplier.getAsDouble();

        double forwardSpeed = -MathUtil.applyDeadband(m_forwardSupplier.getAsDouble(), kJoystickDeadband)
            * m_maxSpeed * speedMultiplier;

        double strafeSpeed = -MathUtil.applyDeadband(m_strafeSupplier.getAsDouble(), kJoystickDeadband)
            * m_maxSpeed * speedMultiplier;

        m_drivetrain.setControl(
            m_robotCentricRequest
                .withVelocityX(forwardSpeed)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(rotationalRate)
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_limelight.setLEDsOff();
        m_drivetrain.setControl(
            m_robotCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}