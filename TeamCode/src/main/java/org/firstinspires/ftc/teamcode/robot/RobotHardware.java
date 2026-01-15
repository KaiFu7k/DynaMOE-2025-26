package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.RobotEnums.ArtifactColor;

/**
 * RobotHardware - Main robot class
 * Aggregates all subsystems and provides single initialization point
 * Used by both Autonomous and TeleOp OpModes
 *
 * This class is the heart of the refactored architecture, bringing together
 * all hardware subsystems in one place for consistent behavior across OpModes.
 */
public class RobotHardware {

    // Subsystems
    public Drivetrain drivetrain;
    public Launcher launcher;
    public Intake intake;
    public ArtifactManager artifactManager;

    // Logger (optional)
    public RobotLogger logger;

    // State
    private boolean isInitialized = false;
    private Telemetry telemetry;

    /**
     * Constructor
     * @param telemetry Telemetry object for all subsystems
     */
    public RobotHardware(Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create logger
        logger = new RobotLogger(telemetry);

        // Initialize subsystems (hardware not connected yet)
        drivetrain = new Drivetrain(telemetry);
        launcher = new Launcher(telemetry);
        intake = new Intake(telemetry);
        artifactManager = new ArtifactManager(telemetry);

        logger.info("RobotHardware", "Subsystems created");
    }

    /**
     * Initialize all hardware subsystems
     * Call this during OpMode init phase
     * @param hardwareMap HardwareMap from OpMode
     */
    public void init(HardwareMap hardwareMap) {
        logger.info("RobotHardware", "Initializing hardware...");

        try {
            // Initialize drivetrain
            drivetrain.init(hardwareMap);
            logger.info("RobotHardware", "Drivetrain initialized");

            // Initialize launcher
            launcher.init(hardwareMap);
            logger.info("RobotHardware", "Launcher initialized");

            // Initialize intake
            intake.init(hardwareMap);
            logger.info("RobotHardware", "Intake initialized");

            // Configure default artifact preload
            artifactManager.configureDefaultPreload();
            logger.info("RobotHardware", "ArtifactManager configured");

            isInitialized = true;

            if (telemetry != null) {
                telemetry.addData("[RobotHardware]", "All systems initialized successfully");
                telemetry.update();
            }

            logger.info("RobotHardware", "Initialization complete!");

        } catch (Exception e) {
            logger.error("RobotHardware", "Initialization failed: " + e.getMessage());
            if (telemetry != null) {
                telemetry.addData("[RobotHardware]", "ERROR: " + e.getMessage());
                telemetry.update();
            }
            throw e;  // Re-throw to stop OpMode
        }
    }

    /**
     * Stop all subsystems
     * Call this at the end of OpMode or when stopping
     */
    public void stopAllSubsystems() {
        logger.info("RobotHardware", "Stopping all subsystems");

        if (drivetrain != null && drivetrain.isInitialized()) {
            drivetrain.stop();
        }

        if (launcher != null && launcher.isInitialized()) {
            launcher.stop();
        }

        if (intake != null && intake.isInitialized()) {
            intake.stop();
        }

        logger.info("RobotHardware", "All subsystems stopped");
    }

    /**
     * Update all subsystems (call in main loop if needed)
     * Currently only launcher needs periodic updates
     */
    public void updateSubsystems() {
        if (launcher != null && launcher.isInitialized()) {
            launcher.update();
        }
    }

    /**
     * Update telemetry for all subsystems
     * Call this to display detailed subsystem status
     */
    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== ROBOT STATUS ===");
        telemetry.addData("Initialized", isInitialized);
        telemetry.addLine();

        if (isInitialized) {
            drivetrain.updateTelemetry();
            launcher.updateTelemetry();
            intake.updateTelemetry();
            artifactManager.updateTelemetry();
        }
    }

    /**
     * Update telemetry with minimal info (for match use)
     */
    public void updateTelemetryCompact() {
        if (telemetry == null) return;

        double[] velocities = launcher.getVelocities();
        telemetry.addData("Launcher", launcher.isReady() ? "READY" : "Spinning");
        telemetry.addData("Artifacts", artifactManager.getTotalArtifacts());
        telemetry.addData("Intake", intake.isRunning() ? "RUNNING" : "Stopped");
    }

    /**
     * Check if all subsystems are initialized
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Get telemetry object
     */
    public Telemetry getTelemetry() {
        return telemetry;
    }

    /**
     * Convenience method: Configure custom artifact preload
     */
    public void configureArtifactPreload(
        java.util.List<ArtifactColor> leftColors,
        java.util.List<ArtifactColor> rightColors) {

        artifactManager.configurePreload(leftColors, rightColors);
        logger.info("RobotHardware", "Custom artifact preload configured");
    }
}
