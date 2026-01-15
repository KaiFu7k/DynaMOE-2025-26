package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotEnums.Motif;

/**
 * MotifDetector utility for detecting MOTIF patterns
 * Encapsulates AprilTag vision detection for OBELISK
 *
 * TODO: Implement AprilTag vision detection
 * For now, supports manual selection
 */
public class MotifDetector {

    // Telemetry
    private Telemetry telemetry;

    // State
    private Motif currentMotif = Motif.GPP;
    private boolean isInitialized = false;

    // Vision hardware (to be implemented)
    // private VisionPortal visionPortal;
    // private AprilTagProcessor aprilTagProcessor;

    /**
     * Constructor
     * @param telemetry Telemetry object for debug output
     */
    public MotifDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initialize vision hardware
     * TODO: Implement AprilTag detection
     */
    public void init(HardwareMap hardwareMap) {
        // TODO: Initialize camera and AprilTag processor
        // visionPortal = new VisionPortal.Builder()
        //     .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        //     .addProcessor(aprilTagProcessor)
        //     .build();

        isInitialized = true;

        if (telemetry != null) {
            telemetry.addData("[MotifDetector]", "Initialized (Manual Mode)");
        }
    }

    /**
     * Detect MOTIF pattern from OBELISK
     * TODO: Replace with AprilTag vision detection
     *
     * IMPLEMENTATION PLAN:
     * 1. Look for OBELISK tags (IDs 21, 22, 23)
     * 2. Read which tag is highlighted/active
     * 3. Map tag ID to MOTIF pattern:
     *    - Tag 21 -> GPP
     *    - Tag 22 -> PGP
     *    - Tag 23 -> PPG
     *
     * @return Detected MOTIF pattern
     */
    public Motif detectMotif() {
        if (!isInitialized) {
            if (telemetry != null) {
                telemetry.addData("[MotifDetector]", "ERROR: Not initialized!");
            }
            return currentMotif;
        }

        // TODO: Implement AprilTag vision detection
        // List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        // for (AprilTagDetection detection : detections) {
        //     if (detection.id == 21) return Motif.GPP;
        //     if (detection.id == 22) return Motif.PGP;
        //     if (detection.id == 23) return Motif.PPG;
        // }

        // For now, return manually set motif
        if (telemetry != null) {
            telemetry.addData("[MotifDetector]", "Manual Mode");
            telemetry.addData("  Detected", currentMotif);
        }

        return currentMotif;
    }

    /**
     * Manually set MOTIF (for testing without vision)
     * @param motif The MOTIF pattern to set
     */
    public void setManualMotif(Motif motif) {
        currentMotif = motif;

        if (telemetry != null) {
            telemetry.addData("[MotifDetector]", "Manual MOTIF set to " + motif);
        }
    }

    /**
     * Cycle to next MOTIF (for gamepad control during init)
     */
    public void cycleMotif() {
        if (currentMotif == Motif.GPP) {
            currentMotif = Motif.PGP;
        } else if (currentMotif == Motif.PGP) {
            currentMotif = Motif.PPG;
        } else {
            currentMotif = Motif.GPP;
        }

        if (telemetry != null) {
            telemetry.addData("[MotifDetector]", "Cycled to " + currentMotif);
        }
    }

    /**
     * Get current MOTIF
     */
    public Motif getCurrentMotif() {
        return currentMotif;
    }

    /**
     * Close vision resources
     */
    public void close() {
        // TODO: Close vision portal when implemented
        // if (visionPortal != null) {
        //     visionPortal.close();
        // }

        if (telemetry != null) {
            telemetry.addData("[MotifDetector]", "Closed");
        }
    }

    /**
     * Check if vision is initialized
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Update telemetry with detector status
     */
    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("[MotifDetector]", "Manual Mode");
        telemetry.addData("  Current MOTIF", currentMotif);
    }
}
