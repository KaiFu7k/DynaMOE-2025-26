package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FieldPositions;

/**
 * LauncherAssist subsystem for auto-alignment and velocity calculation
 */
public class LauncherAssist {

    // Constants
    private static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
    private static final double ROTATION_SPEED = 0.6;  // Increased from 0.5
    private static final double MIN_ROTATION_POWER = 0.18; // Increased from 0.12 to overcome friction

    // PID Constants
    private static final double KP = 1.5;  // Increased from 0.8 for faster response
    private static final double KD = 0.1;  // Increased to maintain damping with higher KP
    private static final double INTEGRAL_MAX = 0.1;

    private static final double[][] VELOCITY_TABLE = {
        {24, 1000}, {36, 1150}, {48, 1250}, {60, 1350}, {72, 1450}, {84, 1550}
    };

    // State
    private Follower follower;
    private Telemetry telemetry;
    private FieldPositions.Alliance currentAlliance;

    private double distanceToGoal = 0;
    private double angleToGoal = 0;
    private double angleError = 0;
    private double recommendedVelocity = 0;
    private boolean isAligned = false;

    private double previousError = 0;
    private double integralSum = 0;
    private long lastUpdateTime = 0;
    private double lastRotationPower = 0;

    public LauncherAssist(Follower follower, FieldPositions.Alliance alliance, Telemetry telemetry) {
        this.follower = follower;
        this.currentAlliance = alliance;
        this.telemetry = telemetry;
        this.lastUpdateTime = System.currentTimeMillis();
    }

    public void update() {
        Pose currentPose = follower.getPose();
        Pose goalPose = FieldPositions.getGoalPosition(currentAlliance);

        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();
        distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        // Standard atan2 (Target Angle)
        angleToGoal = Math.atan2(dy, dx);

        // Standard Error = Target - Current
        angleError = normalizeAngle(angleToGoal - currentPose.getHeading());

        isAligned = Math.abs(Math.toDegrees(angleError)) < ALIGNMENT_TOLERANCE_DEGREES;
        recommendedVelocity = calculateVelocity(distanceToGoal);
    }

    /**
     * Get rotation power. 
     * Note: In our drivetrain, positive rx = Clockwise.
     * To go CCW (positive error), we need negative rx.
     */
    public double getRotationPower() {
        if (isAligned) {
            integralSum = 0;
            previousError = 0;
            return 0.0;
        }

        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        if (dt <= 0 || dt > 0.5) dt = 0.02;

        double proportional = KP * angleError;
        double derivative = KD * (angleError - previousError) / dt;
        previousError = angleError;

        // Negative sign because positive error (CCW needed) requires negative rx power (CCW)
        double rotationPower = -(proportional + derivative);

        rotationPower = Math.max(-ROTATION_SPEED, Math.min(ROTATION_SPEED, rotationPower));

        if (Math.abs(rotationPower) > 0.01 && Math.abs(rotationPower) < MIN_ROTATION_POWER) {
            rotationPower = Math.signum(rotationPower) * MIN_ROTATION_POWER;
        }

        lastRotationPower = rotationPower;
        return rotationPower;
    }

    private double calculateVelocity(double distance) {
        if (distance <= VELOCITY_TABLE[0][0]) return VELOCITY_TABLE[0][1];
        if (distance >= VELOCITY_TABLE[VELOCITY_TABLE.length - 1][0]) return VELOCITY_TABLE[VELOCITY_TABLE.length - 1][1];

        for (int i = 0; i < VELOCITY_TABLE.length - 1; i++) {
            double dist1 = VELOCITY_TABLE[i][0];
            double dist2 = VELOCITY_TABLE[i + 1][0];
            if (distance >= dist1 && distance <= dist2) {
                double vel1 = VELOCITY_TABLE[i][1];
                double vel2 = VELOCITY_TABLE[i + 1][1];
                return vel1 + (distance - dist1) / (dist2 - dist1) * (vel2 - vel1);
            }
        }
        return 1250;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public boolean isAligned() { return isAligned; }
    public double getDistanceToGoal() { return distanceToGoal; }
    public double getAngleErrorDegrees() { return Math.toDegrees(angleError); }
    public double getRecommendedVelocity() { return recommendedVelocity; }
    public void setAlliance(FieldPositions.Alliance alliance) { this.currentAlliance = alliance; }
    public boolean isInLaunchZone() { return FieldPositions.isInLaunchZone(follower.getPose()); }
    public void resetPID() { integralSum = 0; previousError = 0; lastUpdateTime = System.currentTimeMillis(); }
    public double getLastRotationPower() { return lastRotationPower; }
    public double getTargetAngleDegrees() { return Math.toDegrees(angleToGoal); }
    public double getCurrentHeadingDegrees() { return Math.toDegrees(follower.getPose().getHeading()); }
}
