package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.shooter.UptakeSubsystem;

public class RobotContainer {
    // Max speeds
    private final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Speed multiplier levels
    private static final double kSlowSpeed   = 0.5;
    private static final double kNormalSpeed = 0.7;
    private static final double kTurboSpeed  = 1.0;

    // Mutable speed multiplier — single-element array allows mutation inside lambdas
    private final double[] m_speedMultiplier = {kNormalSpeed};

    // Swerve requests
    private final SwerveRequest.FieldCentric m_fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * 0.1)
        .withRotationalDeadband(kMaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.PointWheelsAt m_pointWheels = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentric m_robotCentricNudge = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Controllers
    private final CommandXboxController m_driverController   = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain      = TunerConstants.createDrivetrain();
    public final LimelightSubsystem      limelightSubsystem = new LimelightSubsystem();
    public final FlyWheelSubsystem       flyWheelSubsystem  = new FlyWheelSubsystem();
    public final UptakeSubsystem         uptakeSubsystem    = new UptakeSubsystem();
    public final ClimberSubsystem        climberSubsystem   = new ClimberSubsystem();
    public final IntakeSubsystem         intakeSubsystem    = new IntakeSubsystem();

    // Telemetry
    private final Telemetry m_telemetry = new Telemetry(kMaxSpeed);

    // Auto chooser
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        // Clear any sticky faults on the CTRE PCM
        new Compressor(PneumaticsModuleType.CTREPCM).clearAllStickyFaults();

        limelightSubsystem.setDrivetrain(drivetrain);

        NamedCommands.registerCommand("ShootClose",
            Commands.sequence(
                flyWheelSubsystem.shootAtVelocity(42.5),
                Commands.deadline(
                    Commands.waitSeconds(5.0),
                    uptakeSubsystem.runCommand()
                ),
                Commands.parallel(
                    flyWheelSubsystem.stopCommand(),
                    uptakeSubsystem.stopCommand()
                )
            )
        );
        NamedCommands.registerCommand("ShootFar",
            Commands.sequence(
                flyWheelSubsystem.shootAtVelocity(65.0),
                Commands.deadline(
                    Commands.waitSeconds(5.0),
                    uptakeSubsystem.runCommand()
                ),
                Commands.parallel(
                    flyWheelSubsystem.stopCommand(),
                    uptakeSubsystem.stopCommand()
                )
            )
        );

        NamedCommands.registerCommand("RevShooterClose",
            flyWheelSubsystem.shootAtVelocity(42.5)
        );

        NamedCommands.registerCommand("RevShooterFar",
            flyWheelSubsystem.shootAtVelocity(65.0)
        );
        NamedCommands.registerCommand("Climb",
            climberSubsystem.toggleCommand()
        );
        
        NamedCommands.registerCommand("Dump",
           intakeSubsystem.reverseCommand());

        NamedCommands.registerCommand("ExtendIntake",
            intakeSubsystem.extendCommand()
        );

        NamedCommands.registerCommand("RunIntake",
            Commands.startEnd(
                () -> intakeSubsystem.setSpeed(1.0),
                () -> intakeSubsystem.stop(),
                intakeSubsystem
            )
        );

        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", m_autoChooser);

        configureBindings();

        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Default drive command — field-centric with live speed multiplier
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                m_fieldCentricDrive
                    .withVelocityX(-m_driverController.getLeftY()  * kMaxSpeed * m_speedMultiplier[0])
                    .withVelocityY(-m_driverController.getLeftX()  * kMaxSpeed * m_speedMultiplier[0])
                    .withRotationalRate(-m_driverController.getRightX() * kMaxAngularRate * m_speedMultiplier[0])
            )
        );

        // Idle while disabled so neutral mode is applied correctly
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );

        // Speed control
        m_driverController.leftTrigger()
            .onTrue(Commands.runOnce(() -> m_speedMultiplier[0] = kSlowSpeed))
            .onFalse(Commands.runOnce(() -> m_speedMultiplier[0] = kNormalSpeed));

        m_driverController.rightTrigger()
            .onTrue(Commands.runOnce(() -> m_speedMultiplier[0] = kTurboSpeed))
            .onFalse(Commands.runOnce(() -> m_speedMultiplier[0] = kNormalSpeed));

        // X-brake
        m_driverController.b().whileTrue(drivetrain.applyRequest(() -> m_brake));

        // Point wheels toward stick direction
        m_driverController.x().whileTrue(drivetrain.applyRequest(() ->
            m_pointWheels.withModuleDirection(
                new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX())
            )
        ));

        // D-pad nudge
        m_driverController.povUp().whileTrue(
            drivetrain.applyRequest(() -> m_robotCentricNudge.withVelocityX(0.5).withVelocityY(0))
        );
        m_driverController.povDown().whileTrue(
            drivetrain.applyRequest(() -> m_robotCentricNudge.withVelocityX(-0.5).withVelocityY(0))
        );

        // SysId routines
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading
        m_driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(m_telemetry::telemeterize);

        // Right bumper — AprilTag rotation alignment, driver controls translation
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

        // Operator right trigger — distance-based shot
        m_operatorController.rightTrigger().whileTrue(
            Commands.sequence(
                Commands.defer(() -> {
                    double distance = limelightSubsystem.getDistanceToTargetSafe();
                    return flyWheelSubsystem.shootAtDistance(distance);
                }, Set.of(flyWheelSubsystem)),
                flyWheelSubsystem.waitUntilAtSpeed(),
                uptakeSubsystem.runCommand()
            )
        ).onFalse(
            Commands.parallel(
                flyWheelSubsystem.stopCommand(),
                uptakeSubsystem.stopCommand()
            )
        );
        // Operator left trigger — hold to run intake, release to stop
        m_operatorController.leftTrigger()
            .onTrue(intakeSubsystem.runCommand())
            .onFalse(intakeSubsystem.stopCommand());

        m_operatorController.back()
                .onTrue(intakeSubsystem.reverseCommand())
                .onFalse(intakeSubsystem.stopCommand());

        // Operator select (back) button — toggle intake deploy/retract
        m_operatorController.povUp()
            .onTrue(intakeSubsystem.toggleCommand());



        // Operator right bumper — fixed-speed shot (no limelight)
        m_operatorController.rightBumper().whileTrue(
            Commands.sequence(
                flyWheelSubsystem.shootAtVelocity(50.0),
                flyWheelSubsystem.waitUntilAtSpeed(),
                uptakeSubsystem.runCommand()
            )
        ).onFalse(
            Commands.parallel(
                flyWheelSubsystem.stopCommand(),
                uptakeSubsystem.stopCommand()
            )
        );

        m_operatorController.start()
            .onTrue(climberSubsystem.toggleCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}