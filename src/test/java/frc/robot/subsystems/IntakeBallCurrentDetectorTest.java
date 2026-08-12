package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IntakeBallCurrentDetectorTest {
    private static final double THRESHOLD_AMPS = 35.0;
    private static final double STARTUP_IGNORE_SECONDS = 0.30;
    private static final double DEBOUNCE_SECONDS = 0.12;

    @Test
    void ignoresStartupInrush() {
        IntakeBallCurrentDetector detector = createDetector();

        assertFalse(detector.update(1.00, 60.0, true));
        assertFalse(detector.update(1.29, 60.0, true));
        assertFalse(detector.isDetected());
    }

    @Test
    void detectsCurrentHeldAboveThresholdAfterStartup() {
        IntakeBallCurrentDetector detector = createDetector();

        assertFalse(detector.update(2.00, 10.0, true));
        assertFalse(detector.update(2.30, THRESHOLD_AMPS, true));
        assertFalse(detector.update(2.41, 42.0, true));
        assertTrue(detector.update(2.42, 42.0, true));
    }

    @Test
    void rejectsShortCurrentSpike() {
        IntakeBallCurrentDetector detector = createDetector();

        detector.update(3.00, 5.0, true);
        assertFalse(detector.update(3.31, 45.0, true));
        assertFalse(detector.update(3.40, 45.0, true));
        assertFalse(detector.update(3.41, 20.0, true));
        assertFalse(detector.update(3.60, 20.0, true));
    }

    @Test
    void detectionStaysLatchedUntilCleared() {
        IntakeBallCurrentDetector detector = createDetector();

        detector.update(4.00, 0.0, true);
        detector.update(4.31, 45.0, true);
        assertTrue(detector.update(4.43, 45.0, true));
        assertTrue(detector.update(4.60, 0.0, true));

        detector.clearDetection();
        assertFalse(detector.isDetected());
        assertFalse(detector.update(4.70, 60.0, true));
        assertFalse(detector.update(4.71, 10.0, true));
        assertFalse(detector.update(4.72, 45.0, true));
        assertTrue(detector.update(4.84, 45.0, true));
    }

    @Test
    void stoppingRollerResetsDetection() {
        IntakeBallCurrentDetector detector = createDetector();

        detector.update(5.00, 0.0, true);
        detector.update(5.31, 45.0, true);
        assertTrue(detector.update(5.43, 45.0, true));
        assertFalse(detector.update(5.44, 45.0, false));
        assertFalse(detector.isDetected());
    }

    @Test
    void invalidSamplesCannotCreateDetection() {
        IntakeBallCurrentDetector detector = createDetector();

        assertFalse(detector.update(6.00, Double.NaN, true));
        assertFalse(detector.update(6.31, Double.POSITIVE_INFINITY, true));
        assertFalse(detector.update(Double.NaN, 60.0, true));
    }

    @Test
    void constructorRejectsNegativeTimingOrCurrentValues() {
        assertThrows(
            IllegalArgumentException.class,
            () -> new IntakeBallCurrentDetector(-1.0, 0.30, 0.12));
        assertThrows(
            IllegalArgumentException.class,
            () -> new IntakeBallCurrentDetector(35.0, -0.30, 0.12));
        assertThrows(
            IllegalArgumentException.class,
            () -> new IntakeBallCurrentDetector(35.0, 0.30, -0.12));
    }

    private static IntakeBallCurrentDetector createDetector() {
        return new IntakeBallCurrentDetector(
            THRESHOLD_AMPS,
            STARTUP_IGNORE_SECONDS,
            DEBOUNCE_SECONDS);
    }
}
