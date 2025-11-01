package org.firstinspires.ftc.teamcode.states;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.ArrayList;
import java.util.List;

public class ReadAprilTag {

    private Limelight3A limelight;

    public ReadAprilTag(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public List<AprilTagData> detectAprilTags() {
        List<AprilTagData> aprilTagDataList = new ArrayList<>();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                AprilTagData data = new AprilTagData(
                        fr.getFiducialId(),
                        fr.getFamily(),
                        fr.getTargetXDegrees(),
                        fr.getTargetYDegrees(),
                        botpose
                );
                aprilTagDataList.add(data);
            }
        }

        return aprilTagDataList;
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void startLimelight(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    public static class AprilTagData {
        public final int id;
        public final String family;
        public final double targetX;
        public final double targetY;
        public final Pose3D botpose;

        public AprilTagData(int id, String family, double targetX, double targetY, Pose3D botpose) {
            this.id = id;
            this.family = family;
            this.targetX = targetX;
            this.targetY = targetY;
            this.botpose = botpose;
        }
    }
}