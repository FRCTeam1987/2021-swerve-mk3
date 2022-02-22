package frc.robot.lib;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class DigitalDebouncer {

    private boolean m_wasPreviouslyTripped;
    private double m_timeStamp;
    private final double m_duration;

    public DigitalDebouncer(final double durationInSeconds) {
        m_duration = durationInSeconds;
        m_timeStamp = Timer.getFPGATimestamp();
    }

    public boolean get() {
        return Timer.getFPGATimestamp() - m_timeStamp > m_duration;
    }

    public void periodic(final boolean isCurrentlyTripped) {
        if (!m_wasPreviouslyTripped && isCurrentlyTripped) {
            m_timeStamp = Timer.getFPGATimestamp();
            m_wasPreviouslyTripped = true;
        } else if (!isCurrentlyTripped) {
            m_timeStamp = Timer.getFPGATimestamp();
            m_wasPreviouslyTripped = false;
        }
    }
}