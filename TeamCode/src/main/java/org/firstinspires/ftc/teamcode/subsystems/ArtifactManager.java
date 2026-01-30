package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotEnums.*;
import java.util.ArrayList;
import java.util.List;

/**
 * Tracks artifacts in launcher slots and handles MOTIF patterns.
 */
public class ArtifactManager {

    private Telemetry telemetry;

    /** FIFO queue for artifacts in each launcher slot. */
    public static class ArtifactSlot {
        private List<ArtifactColor> artifacts = new ArrayList<>();

        public void add(ArtifactColor color) { artifacts.add(color); }
        public ArtifactColor removeNext() { return artifacts.isEmpty() ? null : artifacts.remove(0); }
        public boolean isEmpty() { return artifacts.isEmpty(); }
        public int size() { return artifacts.size(); }
        public List<ArtifactColor> getArtifacts() { return new ArrayList<>(artifacts); }
        public void clear() { artifacts.clear(); }
    }

    private ArtifactSlot leftSlot = new ArtifactSlot();
    private ArtifactSlot rightSlot = new ArtifactSlot();

    public ArtifactManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /** Default preload: LEFT=[Purple, Purple], RIGHT=[Green] */
    public void configureDefaultPreload() {
        leftSlot.clear();
        rightSlot.clear();
        leftSlot.add(ArtifactColor.PURPLE);
        leftSlot.add(ArtifactColor.PURPLE);
        rightSlot.add(ArtifactColor.GREEN);
    }

    public void configurePreload(List<ArtifactColor> leftColors, List<ArtifactColor> rightColors) {
        leftSlot.clear();
        rightSlot.clear();
        for (ArtifactColor c : leftColors) leftSlot.add(c);
        for (ArtifactColor c : rightColors) rightSlot.add(c);
    }

    public void addArtifact(LauncherSide side, ArtifactColor color) {
        if (side == LauncherSide.LEFT) leftSlot.add(color);
        else if (side == LauncherSide.RIGHT) rightSlot.add(color);
    }

    public ArtifactColor removeNext(LauncherSide side) {
        if (side == LauncherSide.LEFT) return leftSlot.removeNext();
        if (side == LauncherSide.RIGHT) return rightSlot.removeNext();
        return null;
    }

    /** Find which slot has the target color. Returns null if not found. */
    public LauncherSide findArtifactSide(ArtifactColor targetColor) {
        for (ArtifactColor c : leftSlot.getArtifacts()) if (c == targetColor) return LauncherSide.LEFT;
        for (ArtifactColor c : rightSlot.getArtifacts()) if (c == targetColor) return LauncherSide.RIGHT;
        return null;
    }

    /** Get launch sequence for MOTIF pattern. */
    public ArtifactColor[] getPatternSequence(Motif motif) {
        switch (motif) {
            case GPP: return new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            case PGP: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
            case PPG: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
            default: return new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
        }
    }

    public String getSlotContents(LauncherSide side) {
        ArtifactSlot slot = (side == LauncherSide.LEFT) ? leftSlot : rightSlot;
        StringBuilder sb = new StringBuilder("[");
        List<ArtifactColor> arts = slot.getArtifacts();
        for (int i = 0; i < arts.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(arts.get(i) == ArtifactColor.PURPLE ? "P" : "G");
        }
        return sb.append("]").toString();
    }

    public int getTotalArtifacts() { return leftSlot.size() + rightSlot.size(); }
    public boolean isEmpty() { return leftSlot.isEmpty() && rightSlot.isEmpty(); }
    public ArtifactSlot getLeftSlot() { return leftSlot; }
    public ArtifactSlot getRightSlot() { return rightSlot; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("[Artifacts]", "L:%s R:%s", getSlotContents(LauncherSide.LEFT), getSlotContents(LauncherSide.RIGHT));
    }
}
