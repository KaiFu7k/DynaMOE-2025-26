package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotEnums.Motif;

/**
 * MOTIF pattern detector. Currently manual-only; AprilTag vision TBD.
 * Tag mapping: 21->GPP, 22->PGP, 23->PPG
 */
public class MotifDetector {

    private Telemetry telemetry;
    private Motif currentMotif = Motif.GPP;
    private boolean isInitialized = false;

    public MotifDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        // TODO: Initialize AprilTag vision
        isInitialized = true;
    }

    /** Detect MOTIF. Returns manual selection until vision is implemented. */
    public Motif detectMotif() {
        if (!isInitialized) return currentMotif;
        // TODO: Implement AprilTag detection
        return currentMotif;
    }

    public void setManualMotif(Motif motif) { currentMotif = motif; }

    public void cycleMotif() {
        if (currentMotif == Motif.GPP) currentMotif = Motif.PGP;
        else if (currentMotif == Motif.PGP) currentMotif = Motif.PPG;
        else currentMotif = Motif.GPP;
    }

    public Motif getCurrentMotif() { return currentMotif; }
    public boolean isInitialized() { return isInitialized; }
    public void close() { /* TODO: Close vision portal */ }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("[MotifDetector]", "Manual: %s", currentMotif);
    }
}
