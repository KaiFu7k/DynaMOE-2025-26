package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * RobotLogger utility for structured logging
 * Provides timestamped, categorized logging for debugging
 * Optional but highly recommended for match troubleshooting
 */
public class RobotLogger {

    /**
     * Log levels for filtering
     */
    public enum LogLevel {
        DEBUG,    // Detailed debug information
        INFO,     // General information
        WARNING,  // Warning messages
        ERROR     // Error messages
    }

    // Telemetry
    private Telemetry telemetry;

    // Timer for timestamps
    private ElapsedTime timer = new ElapsedTime();

    // Log history (limited size)
    private List<LogEntry> logHistory = new ArrayList<>();
    private static final int MAX_LOG_HISTORY = 50;

    // Current log level filter
    private LogLevel minLogLevel = LogLevel.INFO;

    // Enable/disable logging
    private boolean enabled = true;

    /**
     * Internal log entry structure
     */
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
            return String.format("[%.2fs] [%s] %s: %s",
                timestamp, level, subsystem, message);
        }
    }

    /**
     * Constructor
     * @param telemetry Telemetry object for output
     */
    public RobotLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
        timer.reset();
    }

    /**
     * Log a message with specified level
     * @param level Log level
     * @param subsystem Subsystem name (e.g., "Drivetrain", "Launcher")
     * @param message Log message
     */
    public void log(LogLevel level, String subsystem, String message) {
        if (!enabled) return;

        // Filter by log level
        if (level.ordinal() < minLogLevel.ordinal()) return;

        LogEntry entry = new LogEntry(timer.seconds(), level, subsystem, message);

        // Add to history
        logHistory.add(entry);
        if (logHistory.size() > MAX_LOG_HISTORY) {
            logHistory.remove(0);  // Remove oldest entry
        }

        // Output to telemetry
        if (telemetry != null) {
            String prefix = getLogPrefix(level);
            telemetry.addData(prefix + subsystem, message);
        }
    }

    /**
     * Log info message
     */
    public void info(String subsystem, String message) {
        log(LogLevel.INFO, subsystem, message);
    }

    /**
     * Log debug message
     */
    public void debug(String subsystem, String message) {
        log(LogLevel.DEBUG, subsystem, message);
    }

    /**
     * Log warning message
     */
    public void warning(String subsystem, String message) {
        log(LogLevel.WARNING, subsystem, message);
    }

    /**
     * Log error message
     */
    public void error(String subsystem, String message) {
        log(LogLevel.ERROR, subsystem, message);
    }

    /**
     * Log formatted message
     */
    public void logf(LogLevel level, String subsystem, String format, Object... args) {
        log(level, subsystem, String.format(format, args));
    }

    /**
     * Get log prefix based on level
     */
    private String getLogPrefix(LogLevel level) {
        switch (level) {
            case DEBUG:   return "[DEBUG] ";
            case INFO:    return "[INFO] ";
            case WARNING: return "[WARN] ";
            case ERROR:   return "[ERROR] ";
            default:      return "";
        }
    }

    /**
     * Set minimum log level for filtering
     */
    public void setMinLogLevel(LogLevel level) {
        minLogLevel = level;
    }

    /**
     * Enable or disable logging
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Get log history
     */
    public List<String> getLogHistory() {
        List<String> history = new ArrayList<>();
        for (LogEntry entry : logHistory) {
            history.add(entry.toString());
        }
        return history;
    }

    /**
     * Clear log history
     */
    public void clearHistory() {
        logHistory.clear();
    }

    /**
     * Reset timer
     */
    public void resetTimer() {
        timer.reset();
    }

    /**
     * Get elapsed time
     */
    public double getElapsedTime() {
        return timer.seconds();
    }

    /**
     * Display recent log history in telemetry
     * @param numEntries Number of recent entries to display
     */
    public void displayHistory(int numEntries) {
        if (telemetry == null) return;

        telemetry.addLine("=== LOG HISTORY ===");
        int startIdx = Math.max(0, logHistory.size() - numEntries);
        for (int i = startIdx; i < logHistory.size(); i++) {
            telemetry.addLine(logHistory.get(i).toString());
        }
    }

    /**
     * Create a subsystem-specific logger
     * Convenience method for subsystems to avoid passing subsystem name every time
     */
    public SubsystemLogger forSubsystem(String subsystem) {
        return new SubsystemLogger(this, subsystem);
    }

    /**
     * Subsystem-specific logger wrapper
     */
    public static class SubsystemLogger {
        private RobotLogger logger;
        private String subsystem;

        public SubsystemLogger(RobotLogger logger, String subsystem) {
            this.logger = logger;
            this.subsystem = subsystem;
        }

        public void debug(String message) { logger.debug(subsystem, message); }
        public void info(String message) { logger.info(subsystem, message); }
        public void warning(String message) { logger.warning(subsystem, message); }
        public void error(String message) { logger.error(subsystem, message); }

        public void logf(LogLevel level, String format, Object... args) {
            logger.logf(level, subsystem, format, args);
        }
    }
}
