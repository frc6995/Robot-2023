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
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeS extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushed);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    Constants.IntakeConstants.INTAKE_EXTEND, Constants.IntakeConstants.INTAKE_RETRACT);
  /** Creates a new IntakeS. */
  public IntakeS() {}

  /**
   * Sets voltage of intake motor to the voltage parameter 
   * @param voltage the voltage supplied to intake motor
   */

  public void intake(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  /**
   * Spins intake motor forward at specified voltage
   */

  public void intake() {
    intake(Constants.IntakeConstants.INTAKE_VOLTAGE);
  }

  /**
   * Spins intake motor in reverse at specified voltage
   */

  public void outtake() {
    intake(-Constants.IntakeConstants.INTAKE_VOLTAGE);
  }

  /**
   * Extends the intake
   */

  public void extend() {
    doubleSolenoid.set(Value.kForward);
  }

  /**
   * Retracts the intake
   */

  public void retract() {
    doubleSolenoid.set(Value.kReverse);
  }

  /**
   * Toggles the intake between extend and retract
   */

  public void toggle() {
    doubleSolenoid.toggle();
  }

  /**
   * Stops the intake motor
   */

  public void stop() {
    intakeMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Runs the intake motor forward and then stops it
   * @return returns the runEnd Command
   */

   public Command intakeC() {
    return runEnd(this::intake, this::stop);
  }

  /**
   * Runs the intake motor in reverse and then stops it
   * @return returns the runEnd Command
   */

  public Command outtakeC() {
    return runEnd(this::outtake, this::stop);
  }

  /**
   * Extends the intake
   * @return returns the runOnce command
   */

  public Command extendC() {
    return runOnce(this::extend);
  }

  /**
   * extends the intake and runs the intake motor forward
   * @return returns the sequence Command
   */

  public Command extendAndIntakeC() {
    return sequence(extendC(), intakeC()).finallyDo((interrupted)-> retract());
  }

  /**
   * extends the intake and runs the intake motor in reverse
   * @return returns the sequence Command
   */

  public Command extendAndOuttakeC() {
    return sequence(extendC(), outtakeC()).finallyDo((interrupted)-> retract());
  }

}
