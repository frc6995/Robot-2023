// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.ArmS.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToPositionC extends CommandBase {
  private ArmS m_armS;
  private ArmPosition m_startPosition;
  private ArmPosition m_targetPosition;
  private List<ArmPosition> m_waypoints;
  private Supplier<ArmPosition> m_positionSupplier;
  private int currentTarget = 0;
  /** Creates a new GoToPositionC. */
  public GoToPositionC(ArmS armS, Supplier<ArmPosition> positionSupplier) {
    m_armS = armS;
    m_positionSupplier = positionSupplier;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_armS);
  }

  private double limitLength(double length) {
    return MathUtil.clamp(length, length, length);
  }
  @Override
  public void initialize() {

    // TODO Auto-generated method stub
    super.initialize();
    currentTarget = 0;
    m_startPosition = m_armS.getArmPosition();
    m_targetPosition = m_positionSupplier.get();

    boolean needToRetract = false;
    boolean needToStraighten = false;

    if (Math.abs(m_startPosition.pivotRadians - m_targetPosition.pivotRadians) > Units.degreesToRadians(10)) {
      needToRetract = true;
    }

    if (Math.abs(m_startPosition.armLength - m_targetPosition.armLength) > Units.inchesToMeters(1)) {
      needToStraighten = true;
    }

    m_armS.resetExtender();
    m_armS.resetPivot();
    m_armS.resetWrist();
    m_waypoints = List.of(
      m_startPosition,
      // rotate the wrist to world-vertical
      new ArmPosition(
        m_startPosition.pivotRadians,
        m_startPosition.armLength,
        needToStraighten ? 
          MathUtil.clamp(m_startPosition.wristRadians, 0, WRIST_MAX_ANGLE) :
          MathUtil.clamp(m_startPosition.wristRadians, WRIST_MIN_ANGLE,  WRIST_MAX_ANGLE),// MathUtil.clamp(Math.PI / 2 - m_startPosition.pivotRadians, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE),
        m_startPosition.handLength),
      new ArmPosition(
        m_startPosition.pivotRadians,
        // before pivot, go to shorter of two lengths
        needToRetract ? MIN_ARM_LENGTH : Math.min(m_startPosition.armLength, m_targetPosition.armLength),
        needToStraighten ? 
          MathUtil.clamp(m_startPosition.wristRadians, 0,  WRIST_MAX_ANGLE) :
          MathUtil.clamp(m_startPosition.wristRadians, WRIST_MIN_ANGLE,  WRIST_MAX_ANGLE),
        m_startPosition.handLength),
      new ArmPosition(
        m_targetPosition.pivotRadians,
        needToRetract ? MIN_ARM_LENGTH : Math.min(m_startPosition.armLength, m_targetPosition.armLength),
        needToStraighten ? 
          MathUtil.clamp(m_targetPosition.wristRadians, 0,  WRIST_MAX_ANGLE) :
          MathUtil.clamp(m_targetPosition.wristRadians, WRIST_MIN_ANGLE,  WRIST_MAX_ANGLE),
        m_startPosition.handLength),
      new ArmPosition(
        m_targetPosition.pivotRadians,
        m_targetPosition.armLength,
        needToStraighten ? 
          MathUtil.clamp(m_targetPosition.wristRadians, 0,  WRIST_MAX_ANGLE) :
          MathUtil.clamp(m_targetPosition.wristRadians, WRIST_MIN_ANGLE,  WRIST_MAX_ANGLE),
        m_startPosition.handLength),
      new ArmPosition(
        m_targetPosition.pivotRadians,
        m_targetPosition.armLength,
        MathUtil.clamp(m_targetPosition.wristRadians, WRIST_MIN_ANGLE,  WRIST_MAX_ANGLE),
        m_startPosition.handLength)
      

    );
  }

  public void execute() {
    if (isAtSetpoint(m_targetPosition)) {
      currentTarget = m_waypoints.size() - 1;
    }
    else if (isAtSetpoint(m_waypoints.get(currentTarget))) {
      if (currentTarget < m_waypoints.size() - 1) {
        currentTarget++;
      }
    }

    var currentTargetPosition = m_waypoints.get(currentTarget);
    m_armS.setExtendLength(currentTargetPosition.armLength);
    m_armS.setPivotAngle(currentTargetPosition.pivotRadians);
    m_armS.setWristAngle(currentTargetPosition.wristRadians);
  }

  private boolean isAtSetpoint (ArmPosition setpoint) {
    
    var actualPosition = m_armS.getArmPosition();
    var atSetpoint = (Math.abs(setpoint.armLength - actualPosition.armLength) < Units.inchesToMeters(2)
      && Math.abs(setpoint.pivotRadians - actualPosition.pivotRadians) < Units.degreesToRadians(5)
      && Math.abs(setpoint.wristRadians - actualPosition.wristRadians) < Units.degreesToRadians(10)
    );
    return atSetpoint;
  }
}