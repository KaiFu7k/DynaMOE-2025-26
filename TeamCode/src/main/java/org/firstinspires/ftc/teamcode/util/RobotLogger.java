package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;

/**
 * Structured logging utility with timestamps and log levels.
 */
public class RobotLogger {

    public enum LogLevel { DEBUG, INFO, WARNING, ERROR }

    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();
    private List<LogEntry> logHistory = new ArrayList<>();
    private static final int MAX_LOG_HISTORY = 50;
    private LogLevel minLogLevel = LogLevel.INFO;
    private boolean enabled = true;

    private static class LogEntry {
        double timestamp;
        LogLevel level;
        String subsystem;
        String message;

        LogEntry(double timestamp, LogLevel level, String subsystem, String message) {
            this.timestamp = timestamp;
            this.level = level;
            this.subsystem = subsystem;
            this.message = message;
        }

        @Override
        public String toString() {
            return String.format("[%.2fs] [%s] %s: %s", timestamp, level, subsystem, message);
        }
    }

    public RobotLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
        timer.reset();
    }

    public void log(LogLevel level, String subsystem, String message) {
        if (!enabled || level.ordinal() < minLogLevel.ordinal()) return;

        LogEntry entry = new LogEntry(timer.seconds(), level, subsystem, message);
        logHistory.add(entry);
        if (logHistory.size() > MAX_LOG_HISTORY) logHistory.remove(0);

        if (telemetry != null) {
            telemetry.addData(getPrefix(level) + subsystem, message);
        }
    }

    public void info(String subsystem, String message) { log(LogLevel.INFO, subsystem, message); }
    public void debug(String subsystem, String message) { log(LogLevel.DEBUG, subsystem, message); }
    public void warning(String subsystem, String message) { log(LogLevel.WARNING, subsystem, message); }
    public void error(String subsystem, String message) { log(LogLevel.ERROR, subsystem, message); }

    public void logf(LogLevel level, String subsystem, String format, Object... args) {
        log(level, subsystem, String.format(format, args));
    }

    private String getPrefix(LogLevel level) {
        switch (level) {
            case DEBUG: return "[D] ";
            case INFO: return "[I] ";
            case WARNING: return "[W] ";
            case ERROR: return "[E] ";
            default: return "";
        }
    }

    public void setMinLogLevel(LogLevel level) { minLogLevel = level; }
    public void setEnabled(boolean enabled) { this.enabled = enabled; }

    public List<String> getLogHistory() {
        List<String> history = new ArrayList<>();
        for (LogEntry entry : logHistory) history.add(entry.toString());
        return history;
    }

    public void clearHistory() { logHistory.clear(); }
    public void resetTimer() { timer.reset(); }
    public double getElapsedTime() { return timer.seconds(); }

    public void displayHistory(int numEntries) {
        if (telemetry == null) return;
        telemetry.addLine("=== LOG HISTORY ===");
        int start = Math.max(0, logHistory.size() - numEntries);
        for (int i = start; i < logHistory.size(); i++) {
            telemetry.addLine(logHistory.get(i).toString());
        }
    }

    public SubsystemLogger forSubsystem(String subsystem) {
        return new SubsystemLogger(this, subsystem);
    }

    public static class SubsystemLogger {
        private RobotLogger logger;
        private String subsystem;

        public SubsystemLogger(RobotLogger logger, String subsystem) {
            this.logger = logger;
            this.subsystem = subsystem;
        }

        public void debug(String msg) { logger.debug(subsystem, msg); }
        public void info(String msg) { logger.info(subsystem, msg); }
        public void warning(String msg) { logger.warning(subsystem, msg); }
        public void error(String msg) { logger.error(subsystem, msg); }
    }
}
