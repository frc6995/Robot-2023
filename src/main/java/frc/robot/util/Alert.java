// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
// Modified by FRC 6995
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

/** Class for managing persistent alerts to be sent over NetworkTables. */
public class Alert {
  private static List<Alert> alerts = new ArrayList<Alert>();
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("DriverDisplay");
  private static StringArrayPublisher errors = table.getStringArrayTopic("errors").publish();
  private static StringArrayPublisher warnings = table.getStringArrayTopic("warnings").publish();
  private static StringArrayPublisher infos = table.getStringArrayTopic("infos").publish();
  private final AlertType type;
  private boolean active = false;
  private double activeStartTime = 0.0;
  private String text;
  private String group;

  /**
   * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated,
   * the appropriate entries will be added to NetworkTables.
   *
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  public Alert(String text, AlertType type) {
    this("", text, type);
  }

  /**
   * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
   * entries will be added to NetworkTables.
   *
   * @param group Group identifier, also used as NetworkTables title
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  public Alert(String group, String text, AlertType type) {
    this.group = group;

    this.type = type;
    alerts.add(this);
  }

  /**
   * Sets whether the alert should currently be displayed. When activated, the alert text will also
   * be sent to the console.
   */
  public void set(boolean active) {
    if (active && !this.active) {
      activeStartTime = Timer.getFPGATimestamp();
      switch (type) {
        case ERROR:
          DriverStation.reportError(text, false);
          break;
        case WARNING:
          DriverStation.reportWarning(text, false);
          break;
        case INFO:
          System.out.println(text);
          break;
      }
    }
    this.active = active;
  }

  private static String[] getStrings(AlertType type) {
    Predicate<Alert> activeFilter = (Alert x) -> x.type == type && x.active;
    Comparator<Alert> timeSorter =
        (Alert a1, Alert a2) -> (int) (a2.activeStartTime - a1.activeStartTime);
    return alerts.stream()
        .filter(activeFilter)
        .sorted(timeSorter)
        .map((Alert a) -> a.group + ": " + a.text)
        .toArray(String[]::new);
  }
  public static void periodic() {
    errors.set(getStrings(AlertType.ERROR));
    warnings.set(getStrings(AlertType.WARNING));
    infos.set(getStrings(AlertType.INFO));
  }


  /** Represents an alert's level of urgency. */
  public static enum AlertType {
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
     * for problems which will seriously affect the robot's functionality and thus require immediate
     * attention.
     */
    ERROR,

    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
     * type for problems which could affect the robot's functionality but do not necessarily require
     * immediate attention.
     */
    WARNING,

    /**
     * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
     * for problems which are unlikely to affect the robot's functionality, or any other alerts
     * which do not fall under "ERROR" or "WARNING".
     */
    INFO
  }
}
