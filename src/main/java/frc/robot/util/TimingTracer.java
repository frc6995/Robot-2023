package frc.robot.util;

import edu.wpi.first.util.WPIUtilJNI;

public class TimingTracer {
    
    private static double m_lastLoopDuration;
    private static double m_lastLoopTimestampSeconds;
    public static void update() {
        m_lastLoopDuration = (WPIUtilJNI.now() / 1e6) - m_lastLoopTimestampSeconds;
        m_lastLoopTimestampSeconds += m_lastLoopDuration;
    }

    public static double getLoopTime() {
        return m_lastLoopDuration;
    }
}
