// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;
import java.util.TreeSet;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import static frc.robot.Constants.LightConstants.*;

public class LightStripS {

  private static LightStripS m_instance = new LightStripS();

  private AddressableLED led = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new  AddressableLEDBuffer(64);
  private final PersistentLedState persistentLedState = new PersistentLedState();

  private static class PersistentLedState {
      public int rainbowFirstPixelHue = 0;
      public int pulseOffset = 0;
  }

  /** Creates a new LedS. */
  private LightStripS() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public static LightStripS getInstance() {
      return m_instance;
  }

  private TreeSet<States> m_states = new TreeSet<>();
  // public double getSpeed() {
  //   return spark.get();
  // }

  /**
   * Different states of the robot, with an integer that determines the priority
   * of the state (the lower the number, the higher the priority)
   */
  public static enum States {
    Disabled(setColor(48, 189, 43)), // set in robotPeriodic
    Error(pulse(0.25, setColor(255, 0, 0))),
    Climbing((ledBuffer, persistentState) -> {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
          final int hue = (persistentState.rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          ledBuffer.setHSV(i, hue, 255, 255);
      }
      persistentState.rainbowFirstPixelHue += 3;
      persistentState.rainbowFirstPixelHue %= 180;
    }), // set through triggers in RobotContainer
    IntakedCone(pulse(0.25, setColor(245, 224, 66))),
    IntakedCube(pulse(0.25, setColor(245, 224, 66))),
    RequestingCube(setColor(186, 15, 172)),
    RequestingCone(setColor(245, 224, 66)),

    Scoring(setColor(0, 0, 255)),
    Default(setColor(0, 255, 0));

    public final BiConsumer<AddressableLEDBuffer, PersistentLedState> setter;

    private States(BiConsumer<AddressableLEDBuffer, PersistentLedState> setter) {
      this.setter = setter;
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
    if (DriverStation.isDisabled()) {
      requestState(States.Disabled);
    }
    //spark.set(m_states.first().lightSpeed);
    m_states.first().setter.accept(buffer, persistentLedState);
    // Do other things with the buffer
    led.setData(buffer);
    m_states.removeAll(Set.of(States.values()));
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> setColor(int r, int g, int b) {
    return (buffer, state)->{
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, r, g, b);
      }
    };
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> pulse(double period, BiConsumer<AddressableLEDBuffer, PersistentLedState> pattern) {
    return (buffer, state)->{
      if (Timer.getFPGATimestamp() % period < 0.5 * period) {
        pattern.accept(buffer, state);
      } else {
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setRGB(i, 0, 0, 0);
        }
      }

    };
  }
}