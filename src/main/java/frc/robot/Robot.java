package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private boolean m_signalLoggerRunning;

    private void startSignalLogger() {
        if (!m_signalLoggerRunning) {
            SignalLogger.start();
            m_signalLoggerRunning = true;
        }
    }

    private void stopSignalLogger() {
        if (m_signalLoggerRunning) {
            SignalLogger.stop();
            m_signalLoggerRunning = false;
        }
    }

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        startSignalLogger();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Robot/BatteryVoltage", voltage);
        SmartDashboard.putBoolean("Robot/BatteryLow",    voltage < 10.5);
    }

    @Override
    public void disabledInit() {
        m_robotContainer.limelightSubsystem.setAutoMode(false);
        m_robotContainer.limelightSubsystem.setLEDsOff();
        stopSignalLogger();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        startSignalLogger();
        m_robotContainer.limelightSubsystem.setAutoMode(true);
        m_robotContainer.limelightSubsystem.setVisionUpdatesEnabled(true);

        // Attempt to hard-reset the starting pose from LL4 vision before auto runs.
        // Falls back to the PathPlanner starting pose if the reset fails.
        m_robotContainer.limelightSubsystem.resetPoseFromVision();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        m_robotContainer.limelightSubsystem.setAutoMode(false);
        m_robotContainer.limelightSubsystem.setVisionUpdatesEnabled(true);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
            // Reseed field-centric heading from the pose auto left us at
            // so the driver doesn't need to press the reset button
            m_robotContainer.drivetrain.seedFieldCentric();
        } else {
            // No auto ran — seed pose from vision so jump filter doesn't reject measurements
            m_robotContainer.limelightSubsystem.resetPoseFromVision();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        startSignalLogger();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}