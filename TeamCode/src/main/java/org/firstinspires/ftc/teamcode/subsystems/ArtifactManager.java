package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotEnums.*;
import java.util.ArrayList;
import java.util.List;

/**
 * ArtifactManager subsystem for tracking and managing artifacts
 * Handles artifact slot tracking and MOTIF pattern matching
 * Used by both Autonomous and TeleOp
 */
public class ArtifactManager {

    // Telemetry
    private Telemetry telemetry;

    /**
     * Queue data structure for tracking artifacts in each launcher slot
     * First artifact added is first to launch (FIFO)
     */
    public static class ArtifactSlot {
        private List<ArtifactColor> artifacts = new ArrayList<>();

        public void add(ArtifactColor color) {
            artifacts.add(color);
        }

        public ArtifactColor removeNext() {
            if (!artifacts.isEmpty()) {
                return artifacts.remove(0);  // Remove from front of queue
            }
            return null;
        }

        public boolean isEmpty() {
            return artifacts.isEmpty();
        }

        public int size() {
            return artifacts.size();
        }

        public List<ArtifactColor> getArtifacts() {
            return new ArrayList<>(artifacts);  // Return copy to prevent modification
        }

        public void clear() {
            artifacts.clear();
        }
    }

    // Artifact slots
    private ArtifactSlot leftSlot = new ArtifactSlot();
    private ArtifactSlot rightSlot = new ArtifactSlot();

    /**
     * Constructor
     * @param telemetry Telemetry object for debug output
     */
    public ArtifactManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Configure default preload (2 Purple + 1 Green)
     * LEFT=[Purple, Purple], RIGHT=[Green]
     */
    public void configureDefaultPreload() {
        leftSlot.clear();
        rightSlot.clear();

        leftSlot.add(ArtifactColor.PURPLE);
        leftSlot.add(ArtifactColor.PURPLE);
        rightSlot.add(ArtifactColor.GREEN);

        if (telemetry != null) {
            telemetry.addData("[ArtifactManager]", "Default preload configured");
        }
    }

    /**
     * Configure custom preload
     * @param leftColors List of colors for left slot (front to back)
     * @param rightColors List of colors for right slot (front to back)
     */
    public void configurePreload(List<ArtifactColor> leftColors, List<ArtifactColor> rightColors) {
        leftSlot.clear();
        rightSlot.clear();

        for (ArtifactColor color : leftColors) {
            leftSlot.add(color);
        }

        for (ArtifactColor color : rightColors) {
            rightSlot.add(color);
        }

        if (telemetry != null) {
            telemetry.addData("[ArtifactManager]", "Custom preload configured");
        }
    }

    /**
     * Add artifact to specified slot
     */
    public void addArtifact(LauncherSide side, ArtifactColor color) {
        if (side == LauncherSide.LEFT) {
            leftSlot.add(color);
        } else if (side == LauncherSide.RIGHT) {
            rightSlot.add(color);
        }
    }

    /**
     * Remove next artifact from specified slot
     */
    public ArtifactColor removeNext(LauncherSide side) {
        if (side == LauncherSide.LEFT) {
            return leftSlot.removeNext();
        } else if (side == LauncherSide.RIGHT) {
            return rightSlot.removeNext();
        }
        return null;
    }

    /**
     * Find which launcher slot contains the target artifact color
     * Searches left slot first, then right slot
     * Returns null if color not found (error condition)
     */
    public LauncherSide findArtifactSide(ArtifactColor targetColor) {
        // Check left slot first
        for (ArtifactColor color : leftSlot.artifacts) {
            if (color == targetColor) {
                return LauncherSide.LEFT;
            }
        }

        // Check right slot
        for (ArtifactColor color : rightSlot.artifacts) {
            if (color == targetColor) {
                return LauncherSide.RIGHT;
            }
        }

        // Artifact color not found in either slot
        return null;
    }

    /**
     * Convert MOTIF enum to launch sequence array
     * Returns the order in which artifacts must be launched (first to last)
     */
    public ArtifactColor[] getPatternSequence(Motif motif) {
        switch (motif) {
            case GPP:
                return new ArtifactColor[] {
                    ArtifactColor.GREEN,
                    ArtifactColor.PURPLE,
                    ArtifactColor.PURPLE
                };
            case PGP:
                return new ArtifactColor[] {
                    ArtifactColor.PURPLE,
                    ArtifactColor.GREEN,
                    ArtifactColor.PURPLE
                };
            case PPG:
                return new ArtifactColor[] {
                    ArtifactColor.PURPLE,
                    ArtifactColor.PURPLE,
                    ArtifactColor.GREEN
                };
            default:
                return new ArtifactColor[] {
                    ArtifactColor.GREEN,
                    ArtifactColor.PURPLE,
                    ArtifactColor.PURPLE
                };
        }
    }

    /**
     * Get string representation of slot contents
     */
    public String getSlotContents(LauncherSide side) {
        ArtifactSlot slot = (side == LauncherSide.LEFT) ? leftSlot : rightSlot;

        StringBuilder sb = new StringBuilder("[");
        List<ArtifactColor> artifacts = slot.getArtifacts();
        for (int i = 0; i < artifacts.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(artifacts.get(i) == ArtifactColor.PURPLE ? "P" : "G");
        }
        sb.append("]");
        return sb.toString();
    }

    /**
     * Get total number of artifacts
     */
    public int getTotalArtifacts() {
        return leftSlot.size() + rightSlot.size();
    }

    /**
     * Check if all slots are empty
     */
    public boolean isEmpty() {
        return leftSlot.isEmpty() && rightSlot.isEmpty();
    }

    /**
     * Get left slot
     */
    public ArtifactSlot getLeftSlot() {
        return leftSlot;
    }

    /**
     * Get right slot
     */
    public ArtifactSlot getRightSlot() {
        return rightSlot;
    }

    /**
     * Update telemetry with artifact manager status
     */
    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("[ArtifactManager]", "");
        telemetry.addData("  Left Slot", getSlotContents(LauncherSide.LEFT));
        telemetry.addData("  Right Slot", getSlotContents(LauncherSide.RIGHT));
        telemetry.addData("  Total", getTotalArtifacts());
    }
}
