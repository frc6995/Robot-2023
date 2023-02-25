// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Robot;

/** Class to control a simulated duty cycle encoder. */
public class DutyCycleEncoderSim {
  private SimDouble m_simPosition;
  private SimDouble m_simAbsolutePosition;
  private SimDouble m_simDistancePerRotation;

  /**
   * Constructs from an DutyCycleEncoder object.
   *
   * @param encoder DutyCycleEncoder to simulate
   */
  public DutyCycleEncoderSim(DutyCycleEncoder encoder) {
    if(Robot.isSimulation()) {
      SimDeviceSim wrappedSimDevice =
      new SimDeviceSim("DutyCycle:DutyCycleEncoder" + "[" + encoder.getSourceChannel() + "]");
  m_simPosition = wrappedSimDevice.getDouble("position");
  m_simAbsolutePosition = wrappedSimDevice.getDouble("absPosition");
  m_simDistancePerRotation = wrappedSimDevice.getDouble("distance_per_rot");
    }
  }

  /**
   * Set the position in turns.
   *
   * @param turns The position.
   */
  public void set(double turns) {
    if(Robot.isSimulation()) {
      m_simPosition.set(turns);
    }

  }

  /**
   * Set the position.
   *
   * @param distance The position.
   */
  public void setDistance(double distance) {
    if(Robot.isSimulation()){
      m_simPosition.set(distance / m_simDistancePerRotation.get());
    }

  }

  public void setAbsolutePosition(double distance) {
    if(Robot.isSimulation()) {
      m_simAbsolutePosition.set(distance / m_simDistancePerRotation.get());
    }

  }

}
