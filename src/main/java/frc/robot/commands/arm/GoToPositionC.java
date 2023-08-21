// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double maxRotateLength = 0.64;
  private int currentTarget = 0;
  private boolean m_shouldFinish = true;
  /** Creates a new GoToPositionC. */
  public GoToPositionC(ArmS armS, Supplier<ArmPosition> positionSupplier) {
    m_armS = armS;
    m_positionSupplier = positionSupplier;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_armS);
  }

  public GoToPositionC(ArmS armS, Supplier<ArmPosition> positionSupplier, boolean shouldEnd) {
    this(armS, positionSupplier);
    m_shouldFinish = shouldEnd;
  }



  private double limitLength(double length) {
    return MathUtil.clamp(length, length, length);
  }
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("armMoving", true);
    // TODO Auto-generated method stub
    super.initialize();
    currentTarget = 0;
    m_startPosition = m_armS.getArmPosition();
    m_targetPosition = m_positionSupplier.get();

    boolean needToRetract = false;
    boolean needToStraighten = false;
    /**
     * 
     */
    double rotateLength = 
    MathUtil.clamp(Math.min(m_startPosition.armLength, m_targetPosition.armLength), MIN_ARM_LENGTH, maxRotateLength);



    if (Math.abs(m_startPosition.armLength - m_targetPosition.armLength) > Units.inchesToMeters(1)) {
      needToStraighten = true;
    }

    m_armS.resetExtender();
    m_armS.resetPivot();
    m_armS.resetWrist();

    m_waypoints = new LinkedList<ArmPosition>();
    m_waypoints.add(m_startPosition);
    {
      double targetLength = m_targetPosition.armLength;
      if (Math.abs(m_startPosition.pivotRadians - m_targetPosition.pivotRadians) > 0.15) {
        needToRetract = true;

      // Step 1, get within the safe length interval for rotating. If we can get to the target length, do it.
      
        double minStartLength = m_armS.getMinLength(m_startPosition.pivotRadians);
        double firstRetractLength = m_startPosition.armLength;
        // if (firstRetractLength > maxRotateLength) {
        //   firstRetractLength = maxRotateLength;
        // }
        // if (targetLength < minStartLength) {
        //   firstRetractLength = minStartLength;
        // }
        firstRetractLength = MathUtil.clamp(firstRetractLength, minStartLength, maxRotateLength);
        targetLength = MathUtil.clamp(targetLength, minStartLength, maxRotateLength);
        
        m_waypoints.add(new ArmPosition(
          m_startPosition.pivotRadians,
          firstRetractLength,
          m_targetPosition.wristRadians
        ));
      }
      
      //step 2, move wrist and pivot to target position. 
      m_waypoints.add(new ArmPosition(
        m_targetPosition.pivotRadians,
        targetLength,
        m_targetPosition.wristRadians));
      //step 3, extend/retract to target length
      m_waypoints.add(m_targetPosition);
    }
    
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
    m_armS.setExtendLength(m_armS.constrainLength(currentTargetPosition.armLength));
    m_armS.setAngle(currentTargetPosition.pivotRadians);
    m_armS.setWristAngle(currentTargetPosition.wristRadians);
  }

  public boolean isFinished() {

    var actualPosition = m_armS.getArmPosition();
    var atSetpoint = (Math.abs(m_armS.constrainLength(m_targetPosition.armLength) - actualPosition.armLength) < Units.inchesToMeters(0.5)
      && Math.abs(m_targetPosition.pivotRadians - actualPosition.pivotRadians) < Units.degreesToRadians(2)
      && Math.abs(m_targetPosition.wristRadians - actualPosition.wristRadians) < Units.degreesToRadians(5)
    );
    return atSetpoint && m_shouldFinish;
  }

  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("armMoving", false);
  }
  private boolean isAtSetpoint (ArmPosition setpoint) {
    
    var actualPosition = m_armS.getArmPosition();
    var atSetpoint = (Math.abs(m_armS.constrainLength(setpoint.armLength) - actualPosition.armLength) < Units.inchesToMeters(2)
      && Math.abs(setpoint.pivotRadians - actualPosition.pivotRadians) < 0.2
      //&& Math.abs(setpoint.wristRadians - actualPosition.wristRadians) < Units.degreesToRadians(10)
    );
    return atSetpoint;
  }
}
