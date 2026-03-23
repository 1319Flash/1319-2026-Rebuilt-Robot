package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AprilTagAlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.UptakeSubsystem;

public class RobotContainer {
    // Maximum speeds — derived from TunerConstants so they stay in sync
    private final double kMaxSpeed       = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Speed multiplier levels — applied on top of kMaxSpeed
    private static final double kSlowSpeed   = 0.5;
    private static final double kNormalSpeed = 0.7;
    private static final double kTurboSpeed  = 1.0;

    // Single-element array so the value can be mutated inside lambdas
    private final double[] m_speedMultiplier = { kNormalSpeed };

    // -------------------------------------------------------------------------
    // Swerve requests — reused every loop to avoid allocations
    // -------------------------------------------------------------------------

    private final SwerveRequest.FieldCentric m_fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * 0.1)
        .withRotationalDeadband(kMaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake m_brake       = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt    m_pointWheels = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentric m_robotCentricNudge = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    private final CommandXboxController m_driverController   = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // -------------------------------------------------------------------------
    // Relays
    // -------------------------------------------------------------------------

    private final Relay m_canivoreRelay = new Relay(0);

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    public final CommandSwerveDrivetrain drivetrain         = TunerConstants.createDrivetrain();
    public final LimelightSubsystem      limelightSubsystem = new LimelightSubsystem();
    public final FlywheelSubsystem       flywheelSubsystem  = new FlywheelSubsystem();
    public final UptakeSubsystem         uptakeSubsystem    = new UptakeSubsystem();
    public final ClimberSubsystem        climberSubsystem   = new ClimberSubsystem();
    public final IntakeSubsystem         intakeSubsystem    = new IntakeSubsystem();

    // Telemetry
    private final Telemetry m_telemetry = new Telemetry(kMaxSpeed);

    // Auto chooser — "Do Nothing" is the safe default if no auto is selected
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        limelightSubsystem.setDrivetrain(drivetrain);

        registerNamedCommands();

        m_autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        SmartDashboard.putData("Auto Mode", m_autoChooser);

        configureBindings();

        // CANivore starts powered on
        m_canivoreRelay.set(Relay.Value.kForward);

        // Pre-warm PathPlanner path following to avoid a stutter on the first auto
        FollowPathCommand.warmupCommand().schedule();
    }

    // =========================================================================
    // Named Commands — registered before AutoBuilder so PathPlanner can find them
    // =========================================================================

    private void registerNamedCommands() {
        // ShootDistance — uses getBestDistanceToTarget() which prefers LL2 (hub-facing)
        // and falls back to LL4 if LL2 has no target. Safe to use from anywhere on the field.
        // Uses waitUntilAtSpeedWithTimeout() so auto doesn't hang if flywheel never reaches speed.
        // Commands.either skips the uptake entirely if the flywheel timed out without reaching speed.
        NamedCommands.registerCommand("ShootDistance",
            Commands.sequence(
                Commands.defer(() -> {
                    double distance = limelightSubsystem.getBestDistanceToTarget();
                    return flywheelSubsystem.shootAtDistance(distance);
                }, Set.of(flywheelSubsystem)),
                flywheelSubsystem.waitUntilAtSpeedWithTimeout(),
                Commands.either(
                    Commands.deadline(
                        Commands.waitSeconds(4.0),
                        uptakeSubsystem.runCommand()
                    ),
                    Commands.none(),
                    flywheelSubsystem::isAtSpeed
                ),
                Commands.parallel(
                    flywheelSubsystem.stopCommand(),
                    uptakeSubsystem.stopCommand()
                )
            )
        );

        // Climb — toggles the climber solenoid
        NamedCommands.registerCommand("Climb",
            climberSubsystem.toggleCommand()
        );

        // ExtendIntake — always deploys the intake regardless of current state.
        // Use extendCommand() not toggleCommand() so this is idempotent in auto.
        NamedCommands.registerCommand("ExtendIntake",
            intakeSubsystem.extendCommand()
        );

        // IntakeBalls — runs intake motor.
        // Use as an event zone in PathPlanner so it runs only while the robot
        // is inside the intake zone and stops automatically when the zone ends.
        NamedCommands.registerCommand("IntakeBalls",
            intakeSubsystem.runCommand()
        );
    }

    // =========================================================================
    // Driver and operator bindings
    // =========================================================================

    private void configureBindings() {
        // Default drive command — field-centric, speed scaled by m_speedMultiplier
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                m_fieldCentricDrive
                    .withVelocityX(-m_driverController.getLeftY()  * kMaxSpeed * m_speedMultiplier[0])
                    .withVelocityY(-m_driverController.getLeftX()  * kMaxSpeed * m_speedMultiplier[0])
                    .withRotationalRate(-m_driverController.getRightX() * kMaxAngularRate * m_speedMultiplier[0])
            )
        );

        // Idle request while disabled — ensures neutral mode is applied correctly
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );

        // -------------------------------------------------------------------------
        // Driver controller
        // -------------------------------------------------------------------------

        // Left trigger — slow speed mode (50%)
        m_driverController.leftTrigger()
            .onTrue(Commands.runOnce(() -> m_speedMultiplier[0] = kSlowSpeed))
            .onFalse(Commands.runOnce(() -> m_speedMultiplier[0] = kNormalSpeed));

        // Right trigger — turbo speed mode (100%)
        m_driverController.rightTrigger()
            .onTrue(Commands.runOnce(() -> m_speedMultiplier[0] = kTurboSpeed))
            .onFalse(Commands.runOnce(() -> m_speedMultiplier[0] = kNormalSpeed));

        // B — X-brake (lock all modules in an X pattern to resist pushing)
        m_driverController.b().whileTrue(drivetrain.applyRequest(() -> m_brake));

        // X — point all wheels toward the current left stick direction
        m_driverController.x().whileTrue(drivetrain.applyRequest(() ->
            m_pointWheels.withModuleDirection(
                new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX())
            )
        ));

        // D-pad — slow robot-centric nudge for fine positioning
        m_driverController.povUp().whileTrue(
            drivetrain.applyRequest(() -> m_robotCentricNudge.withVelocityX(0.5).withVelocityY(0))
        );
        m_driverController.povDown().whileTrue(
            drivetrain.applyRequest(() -> m_robotCentricNudge.withVelocityX(-0.5).withVelocityY(0))
        );

        // SysId — Back/Start + Y/X for translation characterization
        // Run these to get kS and kV for drive motors via the SysId GUI
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // A — reset field-centric heading (point robot's current heading as "forward")
        m_driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(m_telemetry::telemeterize);

        // Right bumper — AprilTag rotation alignment using LL2 tx
        // Driver retains full translation control. Only rotation is automated.
        m_driverController.rightBumper().whileTrue(
            new AprilTagAlignCommand(
                drivetrain,
                limelightSubsystem,
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                kMaxSpeed,
                kMaxAngularRate,
                () -> m_speedMultiplier[0]
            )
        );

        // Back + Left bumper — cut CANivore power to recover from brownout
        // Hold both simultaneously to avoid accidental triggering
        m_driverController.back().and(m_driverController.leftBumper())
            .onTrue(Commands.runOnce(() -> m_canivoreRelay.set(Relay.Value.kOff)));

        // Back + Right bumper — restore CANivore power
        m_driverController.back().and(m_driverController.rightBumper())
            .onTrue(Commands.runOnce(() -> m_canivoreRelay.set(Relay.Value.kForward)));

        // -------------------------------------------------------------------------
        // Operator controller
        // -------------------------------------------------------------------------

        // Right trigger — distance-based shot using getBestDistanceToTarget()
        // No timeout on waitUntilAtSpeed() — driver releases button to cancel.
        m_operatorController.rightTrigger().whileTrue(
            Commands.sequence(
                Commands.defer(() -> {
                    double distance = limelightSubsystem.getBestDistanceToTarget();
                    return flywheelSubsystem.shootAtDistance(distance);
                }, Set.of(flywheelSubsystem)),
                flywheelSubsystem.waitUntilAtSpeed(),
                uptakeSubsystem.runCommand()
            )
        ).onFalse(
            Commands.parallel(
                flywheelSubsystem.stopCommand(),
                uptakeSubsystem.stopCommand()
            )
        );

        // Left bumper — hold to reverse uptake for unjamming
        m_operatorController.leftBumper()
            .whileTrue(uptakeSubsystem.reverseCommand())
            .onFalse(uptakeSubsystem.stopCommand());

        // Right bumper — fixed velocity shot for PID tuning
        m_operatorController.rightBumper()
            .onTrue(
                Commands.sequence(
                    flywheelSubsystem.shootAtVelocity(30.0),
                    flywheelSubsystem.waitUntilAtSpeed(),
                    uptakeSubsystem.runCommand()
                )
            )
            .onFalse(
                Commands.parallel(
                    flywheelSubsystem.stopCommand(),
                    uptakeSubsystem.stopCommand()
                )
            );

        // Left trigger — run intake to collect a ball
        m_operatorController.leftTrigger()
            .whileTrue(intakeSubsystem.runCommand())
            .onFalse(intakeSubsystem.stopCommand());

        // Back — manual intake reverse for unjamming
        m_operatorController.back()
            .onTrue(intakeSubsystem.reverseCommand())
            .onFalse(intakeSubsystem.stopCommand());

        // POV Up — toggle intake deploy/retract
        m_operatorController.povUp()
            .onTrue(intakeSubsystem.toggleCommand());

        // Start — toggle climber
        m_operatorController.start()
            .onTrue(climberSubsystem.toggleCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}