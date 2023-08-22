// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import static frc.robot.Constants.LightConstants.*;

public class LightS {

  private static LightS m_instance = new LightS();

  private Spark spark = new Spark(PWM_PORT_LED);


  /** Creates a new LedS. */
  private LightS() {
  }

  public static LightS getInstance() {
      return m_instance;
  }

  private TreeSet<States> m_states = new TreeSet<>();
  public double getSpeed() {
    return spark.get();
  }

  /**
   * Different states of the robot, with an integer that determines the priority
   * of the state (the lower the number, the higher the priority)
   */
  public static enum States {
    Disabled(LED_BOUNCE_GREEN), // set in robotPeriodic
    Error(LED_LIGHT_CHASE_RED),
    Climbing(LED_PARTY_MODE), // set through triggers in RobotContainer
    IntakedCone(LED_STROBE_GOLD),
    IntakedCube(LED_STROBE_GOLD),
    RequestingCube(LED_SOLID_VIOLET),
    RequestingCone(LED_SOLID_YELLOW),

    Scoring(LED_SINELON_OCEAN),
    Default(LED_SINELON_OCEAN);

    public final double lightSpeed;

    private States(double lightSpeed) {
      this.lightSpeed = lightSpeed;
    }
  }

  // currentStates = {Disabled, Climbing, EjectingWrongColor, Intaking, Shooting,
  // Default}

  /**
   * Requests the current state of the robot, determines whether the requested
   * state is a higher priority than the current state, sets the current state to
   * the requested state
   * 
   * @param state The requested state of the robot when the method is called
   */
  public void requestState(States state) {
    m_states.add(state);
  }

  /**
   * Periodically checks the current state of the robot and sets the LEDs to the
   * corresponding light pattern
   */
  public void periodic() {
    requestState(States.Default);
    spark.set(m_states.first().lightSpeed);
    m_states.removeAll(Set.of(States.values()));
  }
}