package frc.robot.subsystems;

/**
 * Detects a likely FUEL intake event from sustained roller-motor current.
 *
 * <p>The detector ignores startup inrush, debounces the current threshold, and
 * latches a detection until the next collection cycle explicitly clears it.
 */
public final class IntakeBallCurrentDetector {
    private static final double TIME_EPSILON_SECONDS = 1e-9;

    private final double currentThresholdAmps;
    private final double startupIgnoreSeconds;
    private final double debounceSeconds;

    private boolean collecting = false;
    private boolean detected = false;
    private boolean armed = true;
    private double collectionStartSeconds = -1.0;
    private double overThresholdStartSeconds = -1.0;

    public IntakeBallCurrentDetector(
            double currentThresholdAmps,
            double startupIgnoreSeconds,
            double debounceSeconds) {
        if (currentThresholdAmps < 0.0  || startupIgnoreSeconds < 0.0 || debounceSeconds < 0.0) {
                throw new IllegalArgumentException("Ball-current detector values must be nonnegative");
        }

        this.currentThresholdAmps = currentThresholdAmps;
        this.startupIgnoreSeconds = startupIgnoreSeconds;
        this.debounceSeconds = debounceSeconds;
    }

    /** Update the detector and return the latched detection state. */
    public boolean update(double nowSeconds, double currentAmps, boolean collectingCommanded) {
        if (!Double.isFinite(nowSeconds)) {
            resetInactive();
            return false;
        }

        if (!collectingCommanded) {
            resetInactive();
            return false;
        }

        if (!collecting || nowSeconds < collectionStartSeconds) {
            collecting = true;
            detected = false;
            armed = true;
            collectionStartSeconds = nowSeconds;
            overThresholdStartSeconds = -1.0;
        }

        if (detected) {
            return true;
        }

        if (nowSeconds - collectionStartSeconds + TIME_EPSILON_SECONDS
                < startupIgnoreSeconds) {
            overThresholdStartSeconds = -1.0;
            return false;
        }

        double safeCurrentAmps = Double.isFinite(currentAmps) ? Math.max(0.0, currentAmps) : 0.0;

        // After one latched event is consumed, current must return below the
        // threshold before another FUEL can create a new event. A sustained jam
        // therefore cannot be counted repeatedly as several collected FUEL.
        if (!armed) {
            if (safeCurrentAmps < currentThresholdAmps) {
                armed = true;
            }
            overThresholdStartSeconds = -1.0;
            return false;
        }

        if (safeCurrentAmps < currentThresholdAmps) {
            overThresholdStartSeconds = -1.0;
            return false;
        }

        if (overThresholdStartSeconds < 0.0) {
            overThresholdStartSeconds = nowSeconds;
        }

        if (nowSeconds - overThresholdStartSeconds + TIME_EPSILON_SECONDS
                >= debounceSeconds) {
            detected = true;
        }

        return detected;
    }

    /** Start a fresh detection window while leaving the roller command unchanged. */
    public void clearDetection() {
        detected = false;
        armed = !collecting;
        overThresholdStartSeconds = -1.0;
    }

    public boolean isDetected() {
        return detected;
    }

    public boolean isAboveThresholdDebouncing() {
        return overThresholdStartSeconds >= 0.0 && !detected;
    }

    private void resetInactive() {
        collecting = false;
        detected = false;
        armed = true;
        collectionStartSeconds = -1.0;
        overThresholdStartSeconds = -1.0;
    }
}
