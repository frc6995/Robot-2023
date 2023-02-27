// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.ArmS.ArmPosition;

public class HoldCurrentPositionC extends CommandBase {
  ArmS m_armS;
  ArmPosition position;
  /** Creates a new HoldCurrentPositionC. */
  public HoldCurrentPositionC(ArmS armS) {
    m_armS = armS;
    addRequirements(m_armS);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = m_armS.getArmPosition();
    m_armS.resetExtender();
    m_armS.resetPivot();
    m_armS.resetWrist();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armS.setExtendLength(position.armLength);
    m_armS.setPivotAngle(position.pivotRadians);
    m_armS.setWristAngle(position.wristRadians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
