package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceWrapper {
    private static Alliance currentAlliance = Alliance.Invalid;
    public static Alliance getAlliance() {
        return currentAlliance;
    }
    public static void setAlliance(Alliance newAlliance) {
        currentAlliance = newAlliance;
    }
}
