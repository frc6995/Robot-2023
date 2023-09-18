package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceWrapper {
    private static Alliance currentAlliance = Alliance.Invalid;
    public static Alliance getAlliance() {
        currentAlliance = DriverStation.getAlliance();
        return currentAlliance;
    }
    public static void setAlliance(Alliance newAlliance) {
        currentAlliance = newAlliance;
    }
    public static boolean isRed() {
        return currentAlliance == Alliance.Red;
    }

    public static boolean isBlue() {
        return currentAlliance == Alliance.Blue;
    }
}
