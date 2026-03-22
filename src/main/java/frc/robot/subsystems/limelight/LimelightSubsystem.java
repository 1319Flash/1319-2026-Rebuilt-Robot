package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.RawFiducial;

/**
 * Manages all vision functions for the robot using a single LL4.
 *
 * The LL4 faces forward toward the hub, providing:
 *   - MegaTag2 pose estimation from hub AprilTags
 *   - tx-based rotation alignment to hub tags
 *   - distToCamera for shooter velocity calculations
 *
 * When the LL2 is added in Phase 2, this file will be updated to use both cameras.
 */
public class LimelightSubsystem extends SubsystemBase {

    // LL4 — forward-facing, single camera for pose, alignment, and distance
    private static final String kLimelightName = "limelight-flash";

    // Proportional gain for tx-based rotation alignment (right bumper command)
    // Tune on carpet: increase if robot doesn't rotate, decrease if it oscillates
    private static final double kPAim = 0.06;

    // -------------------------------------------------------------------------
    // Pose rejection thresholds — loosened for 2026 field limited tag visibility
    // -------------------------------------------------------------------------
    private static final double kMaxAmbiguity         = 0.4;
    private static final double kMaxSingleTagDistance = 5.5;  // meters
    private static final double kMaxMultiTagDistance  = 7.0;  // meters
    private static final double kMaxPoseJumpTeleop    = 1.5;  // meters
    private static final double kMaxPoseJumpAuto      = 1.0;  // meters
    private static final double kMaxRotationDegPerSec = 720.0; // deg/s

    // -------------------------------------------------------------------------
    // 2026 field dimensions (meters)
    // -------------------------------------------------------------------------
    private static final double kFieldLengthMeters = 17.548;
    private static final double kFieldWidthMeters  = 8.211;
    private static final double kFieldMarginMeters = 0.5;

    // -------------------------------------------------------------------------
    // Standard deviation base values — higher = less trust in vision
    // -------------------------------------------------------------------------
    private static final double kSingleTagXyBase    = 0.5;
    private static final double kSingleTagThetaBase = 0.9;
    private static final double kMultiTagXyBase     = 0.2;
    private static final double kMultiTagThetaBase  = 0.4;
    private static final double kDistanceExp        = 2.0;

    // Frames to skip after mode change before accepting corrections (~0.5s at 50Hz)
    private static final int kWarmupFrames = 25;

    // How long a pose stays confident after the last accepted measurement
    private static final double kPoseConfidenceWindowSeconds = 2.0;

    // -------------------------------------------------------------------------
    // Auto start pose reset
    // -------------------------------------------------------------------------
    private static final int    kAutoResetMinTags  = 1;
    private static final double kAutoResetMaxDist  = 5.0;
    private static final double kAutoResetMaxAmbig = 0.25;

    // -------------------------------------------------------------------------
    // Hub tag IDs — only accept these for distance calculations so non-hub
    // field tags never corrupt shooter velocity
    // -------------------------------------------------------------------------
    private static final java.util.Set<Integer> kHubTagIds = new java.util.HashSet<>(
        java.util.Arrays.asList(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27)
    );

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private CommandSwerveDrivetrain m_drivetrain;
    private boolean m_visionUpdatesEnabled = true;
    private boolean m_isAutoMode           = false;

    private double m_lastYawDegrees       = 0.0;
    private double m_lastTimestamp        = 0.0;
    private double m_rotDegPerSec         = 0.0;
    private int    m_frameCount           = 0;
    private double m_lastVisionUpdateTime = 0.0;

    // Cached estimate — reused for Field2d widget update
    private LimelightHelpers.PoseEstimate m_latestMt2 = null;

    // Field2d widget — shows raw vision pose alongside odometry pose in Elastic
    private final Field2d m_visionField = new Field2d();

    public LimelightSubsystem() {
        setLEDsOff();
        SmartDashboard.putData("Vision/Field", m_visionField);
    }

    // =========================================================================
    // Public setters
    // =========================================================================

    /** Sets the drivetrain reference required for MegaTag2 odometry fusion. */
    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    /** Enables or disables vision odometry updates without affecting telemetry. */
    public void setVisionUpdatesEnabled(boolean enabled) {
        m_visionUpdatesEnabled = enabled;
    }

    /** Switches auto/teleop thresholds and controls LEDs. */
    public void setAutoMode(boolean isAuto) {
        m_isAutoMode = isAuto;
        m_frameCount = 0;
        if (isAuto) {
            setLEDsOn();
        } else {
            setLEDsOff();
        }
    }

    // =========================================================================
    // Getters
    // =========================================================================

    /** Returns horizontal offset from crosshair to target in degrees. */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(kLimelightName);
    }

    /** Returns true if the LL4 has a valid target lock. */
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(kLimelightName);
    }

    /**
     * Returns distance from the LL4 lens to the primary hub AprilTag in meters.
     * Only accepts hub tag IDs to prevent non-hub tags from corrupting distance.
     *
     * @return distance in meters, or -1.0 if no hub tag is visible
     */
    public double getDistanceToTarget() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(kLimelightName);
        for (RawFiducial fiducial : fiducials) {
            if (kHubTagIds.contains(fiducial.id)) {
                double d = fiducial.distToCamera;
                if (d > 0 && d <= 10.0 && !Double.isNaN(d)) return d;
            }
        }
        return -1.0;
    }

    /**
     * Returns distance to hub tag, falling back to 1.0 m if none visible.
     * Used by shooter velocity calculations.
     */
    public double getBestDistanceToTarget() {
        double d = getDistanceToTarget();
        if (d < 0) return 1.0;
        return d;
    }

    /**
     * Returns a rotational rate for aligning to the current hub tag using tx.
     *
     * @param maxAngularRate the robot's maximum angular rate in radians per second
     * @return rotational rate in rad/s, or 0 if no target is visible
     */
    public double calculateAimVelocity(double maxAngularRate) {
        if (!hasValidTarget()) return 0.0;
        return -LimelightHelpers.getTX(kLimelightName) * kPAim * maxAngularRate;
    }

    /** Returns the latest cached MegaTag2 pose estimate. Updated once per loop. */
    public LimelightHelpers.PoseEstimate getMegaTag2Estimate() {
        return m_latestMt2;
    }

    /** Returns true if a vision measurement was accepted within the last 2 seconds. */
    public boolean isPoseConfident() {
        return Timer.getFPGATimestamp() - m_lastVisionUpdateTime < kPoseConfidenceWindowSeconds;
    }

    // =========================================================================
    // LED control
    // =========================================================================

    public void setLEDsOn()  { LimelightHelpers.setLEDMode_ForceOn(kLimelightName);  }
    public void setLEDsOff() { LimelightHelpers.setLEDMode_ForceOff(kLimelightName); }

    // Kept for compatibility with AprilTagAlignCommand — both point to the same camera
    public void setRearLEDsOn()   { setLEDsOn();  }
    public void setRearLEDsOff()  { setLEDsOff(); }
    public void setFrontLEDsOn()  { setLEDsOn();  }
    public void setFrontLEDsOff() { setLEDsOff(); }

    /** Switches the LL4 pipeline by index. */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(kLimelightName, pipeline);
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    private String m_lastStatus = "";

    private void putStatus(String msg) {
        m_lastStatus = msg;
    }

    /**
     * Sends gyro yaw to the LL4 and tracks rotation speed.
     * Required before reading MegaTag2 estimate each loop.
     */
    private void seedOrientation() {
        if (m_drivetrain == null) return;

        double yaw = m_drivetrain.getState().Pose.getRotation().getDegrees();
        double now = Timer.getFPGATimestamp();

        if (m_lastTimestamp > 0.0) {
            double dt = now - m_lastTimestamp;
            if (dt > 0.0) {
                double delta = yaw - m_lastYawDegrees;
                while (delta >  180.0) delta -= 360.0;
                while (delta < -180.0) delta += 360.0;
                m_rotDegPerSec = Math.abs(delta / dt);
            }
        }

        m_lastYawDegrees = yaw;
        m_lastTimestamp  = now;

        LimelightHelpers.SetRobotOrientation(kLimelightName, yaw, 0, 0, 0, 0, 0);
    }

    /** Returns true if the estimate passes all rejection criteria. */
    private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate est) {
        if (est.tagCount == 0) {
            putStatus("No tags");
            return false;
        }

        if (m_frameCount < kWarmupFrames) {
            putStatus("Warmup " + m_frameCount + "/" + kWarmupFrames);
            return false;
        }

        if (m_rotDegPerSec > kMaxRotationDegPerSec) {
            putStatus("Spinning");
            return false;
        }

        if (est.tagCount == 1) {
            if (est.rawFiducials.length > 0 && est.rawFiducials[0].ambiguity > kMaxAmbiguity) {
                putStatus("Ambiguity high");
                return false;
            }
            if (est.avgTagDist > kMaxSingleTagDistance) {
                putStatus("Tag too far");
                return false;
            }
        } else if (est.avgTagDist > kMaxMultiTagDistance) {
            putStatus("Tags too far");
            return false;
        }

        double px = est.pose.getX();
        double py = est.pose.getY();

        if (px < -kFieldMarginMeters || px > kFieldLengthMeters + kFieldMarginMeters
                || py < -kFieldMarginMeters || py > kFieldWidthMeters + kFieldMarginMeters) {
            putStatus("Off field");
            return false;
        }

        if (px == 0.0 && py == 0.0) {
            putStatus("At origin");
            return false;
        }

        if (m_drivetrain != null) {
            Pose2d odom    = m_drivetrain.getState().Pose;
            double jump    = odom.getTranslation().getDistance(est.pose.getTranslation());
            double maxJump = m_isAutoMode ? kMaxPoseJumpAuto : kMaxPoseJumpTeleop;
            if (jump > maxJump) {
                putStatus("Jump " + String.format("%.1f", jump) + "m");
                return false;
            }
        }

        putStatus("OK " + est.tagCount + "t");
        return true;
    }

    /** Computes Kalman filter standard deviations scaled by distance and tag count. */
    private double[] computeStdDevs(LimelightHelpers.PoseEstimate est) {
        double distFactor = Math.pow(est.avgTagDist, kDistanceExp);
        double xy, theta;

        if (est.tagCount >= 2) {
            xy    = kMultiTagXyBase    * distFactor;
            theta = kMultiTagThetaBase * distFactor;
            if (est.tagCount >= 3) {
                xy    *= 0.5;
                theta *= 0.5;
            }
        } else {
            double ambig       = est.rawFiducials.length > 0 ? est.rawFiducials[0].ambiguity : kMaxAmbiguity;
            double ambigFactor = 1.0 + (ambig * 3.0);
            xy    = kSingleTagXyBase    * distFactor * ambigFactor;
            theta = kSingleTagThetaBase * distFactor * ambigFactor;
        }

        return new double[]{ Math.min(xy, 10.0), Math.min(theta, 10.0) };
    }

    /** Runs the vision measurement pipeline. */
    private void processVisionMeasurements() {
        if (m_drivetrain == null || !m_visionUpdatesEnabled) return;

        m_frameCount++;
        seedOrientation();

        m_latestMt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (m_latestMt2 == null || !isValidVisionMeasurement(m_latestMt2)) return;

        double[] stdDevs = computeStdDevs(m_latestMt2);
        m_drivetrain.addVisionMeasurement(
            m_latestMt2.pose,
            m_latestMt2.timestampSeconds,
            stdDevs[0],
            stdDevs[1]
        );

        m_lastVisionUpdateTime = Timer.getFPGATimestamp();
    }

    // =========================================================================
    // Pose reset
    // =========================================================================

    /**
     * Hard pose reset using LL4 MegaTag2 estimate.
     * Called from autonomousInit() and teleopInit().
     */
    public boolean resetPoseFromVision() {
        if (m_drivetrain == null) {
            SmartDashboard.putString("AutoStart/PoseReset", "No drivetrain");
            return false;
        }

        double yaw = m_drivetrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(kLimelightName, yaw, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate est =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (est == null || est.tagCount < kAutoResetMinTags) {
            SmartDashboard.putString("AutoStart/PoseReset", "No tags");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        if (est.avgTagDist > kAutoResetMaxDist) {
            SmartDashboard.putString("AutoStart/PoseReset", "Too far");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        if (est.rawFiducials.length > 0 && est.rawFiducials[0].ambiguity > kAutoResetMaxAmbig) {
            SmartDashboard.putString("AutoStart/PoseReset", "High ambiguity");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        double px = est.pose.getX();
        double py = est.pose.getY();

        if (px < -kFieldMarginMeters || px > kFieldLengthMeters + kFieldMarginMeters
                || py < -kFieldMarginMeters || py > kFieldWidthMeters + kFieldMarginMeters
                || (px == 0.0 && py == 0.0)) {
            SmartDashboard.putString("AutoStart/PoseReset", "Invalid pose");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        m_drivetrain.resetPose(est.pose);
        m_frameCount = 0;

        SmartDashboard.putString("AutoStart/PoseReset", "OK");
        SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", true);

        return true;
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        try {
            processVisionMeasurements();

            SmartDashboard.putString("Vision/Status",   m_lastStatus);
            SmartDashboard.putBoolean("Vision/Active",  isPoseConfident());
            SmartDashboard.putBoolean("Vision/HasTarget", hasValidTarget());
            SmartDashboard.putNumber("Vision/Distance", getBestDistanceToTarget());

            if (m_latestMt2 != null && m_latestMt2.tagCount > 0) {
                m_visionField.setRobotPose(m_latestMt2.pose);
            }
        } catch (Exception e) {
            SmartDashboard.putString("Vision/Error", e.getMessage());
        }
    }
}