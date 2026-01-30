package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.RobotEnums.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.FieldPositions;

/**
 * Main robot class that aggregates all subsystems.
 * Used by both Autonomous and TeleOp OpModes.
 */
public class RobotHardware {

    public Drivetrain drivetrain;
    public Launcher launcher;
    public Intake intake;
    public ArtifactManager artifactManager;
    public LauncherAssist launcherAssist;
    public RobotLogger logger;

    private boolean isInitialized = false;
    private Telemetry telemetry;

    public RobotHardware(Telemetry telemetry) {
        this.telemetry = telemetry;
        logger = new RobotLogger(telemetry);
        drivetrain = new Drivetrain(telemetry);
        launcher = new Launcher(telemetry);
        intake = new Intake(telemetry);
        artifactManager = new ArtifactManager(telemetry);
    }

    /** Initialize all hardware. Call during OpMode init. */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, null, FieldPositions.Alliance.BLUE);
    }

    /** Initialize all hardware with LauncherAssist. */
    public void init(HardwareMap hardwareMap, Follower follower, FieldPositions.Alliance alliance) {
        try {
            drivetrain.init(hardwareMap);
            launcher.init(hardwareMap);
            intake.init(hardwareMap);
            artifactManager.configureDefaultPreload();

            if (follower != null) {
                launcherAssist = new LauncherAssist(follower, alliance, telemetry);
            }

            isInitialized = true;
            if (telemetry != null) {
                telemetry.addData("[RobotHardware]", "Initialized");
                telemetry.update();
            }
        } catch (Exception e) {
            logger.error("RobotHardware", "Init failed: " + e.getMessage());
            throw e;
        }
    }

    /** Stop all subsystems. */
    public void stopAllSubsystems() {
        if (drivetrain != null && drivetrain.isInitialized()) drivetrain.stop();
        if (launcher != null && launcher.isInitialized()) launcher.stop();
        if (intake != null && intake.isInitialized()) intake.stop();
    }

    /** Update subsystems that need periodic updates. */
    public void updateSubsystems() {
        if (launcher != null && launcher.isInitialized()) launcher.update();
        if (launcherAssist != null) launcherAssist.update();
    }

    /** Display detailed subsystem status. */
    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addLine("=== ROBOT STATUS ===");
        telemetry.addData("Initialized", isInitialized);
        if (isInitialized) {
            drivetrain.updateTelemetry();
            launcher.updateTelemetry();
            intake.updateTelemetry();
            artifactManager.updateTelemetry();
        }
    }

    /** Display compact telemetry for matches. */
    public void updateTelemetryCompact() {
        if (telemetry == null) return;
        telemetry.addData("Launcher", launcher.isReady() ? "READY" : "Spinning");
        telemetry.addData("Artifacts", artifactManager.getTotalArtifacts());
        telemetry.addData("Intake", intake.isRunning() ? "RUNNING" : "Stopped");
    }

    public boolean isInitialized() { return isInitialized; }
    public Telemetry getTelemetry() { return telemetry; }

    public void configureArtifactPreload(java.util.List<ArtifactColor> leftColors,
                                          java.util.List<ArtifactColor> rightColors) {
        artifactManager.configurePreload(leftColors, rightColors);
    }
}
