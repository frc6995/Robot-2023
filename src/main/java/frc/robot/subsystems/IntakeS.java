// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.IntakeC;

public class IntakeS extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushed);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    Constants.IntakeConstants.INTAKE_EXTEND, Constants.IntakeConstants.INTAKE_RETRACT);
  /** Creates a new IntakeS. */
  public IntakeS() {}

  public void intake(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public void intake() {
    intake(Constants.IntakeConstants.INTAKE_VOLTAGE);
  }

  public void outtake() {
    intakeMotor.setVoltage(-Constants.IntakeConstants.INTAKE_VOLTAGE);
  }

  public void extend() {
    doubleSolenoid.set(Value.kForward);
  }

  public void retract() {
    doubleSolenoid.set(Value.kReverse);
  }

  public void toggle() {
    doubleSolenoid.toggle();
  }

  public void stop() {
    intakeMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command intakeC() {
    return runEnd(this::intake, this::stop);
  }

  public Command outtakeC() {
    return runEnd(this::outtake, this::stop);
  }

  public Command extendC() {
    return runOnce(this::extend);
  }

  public Command extendAndIntakeC() {
    return Commands.sequence(extendC(), intakeC()).finallyDo((interrupted)-> retract());
  }

  public Command extendAndOuttakeC() {
    return Commands.sequence(extendC(), outtakeC()).finallyDo((interrupted)-> retract());
  }

}
