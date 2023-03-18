// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import frc.robot.util.color.PicoColorSensor.RawColor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.color.PicoColorSensor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeS extends SubsystemBase implements Loggable {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax intakeFollowerMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_FOLLOWER_CAN_ID, MotorType.kBrushless);
  private final SparkMaxLimitSwitch m_beamBreak = intakeFollowerMotor.getReverseLimitSwitch(Type.kNormallyClosed);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
    Constants.IntakeConstants.INTAKE_EXTEND, Constants.IntakeConstants.INTAKE_RETRACT);
  private Trigger cubeDebouncedBeamBreak = new Trigger(this::hitBeamBreak).debounce(0.06);
  private Trigger coneDebouncedBeamBreak = new Trigger(this::hitBeamBreak);//.debounce(0.0);
    /** Creates a new IntakeS. */
  public IntakeS() {
    m_beamBreak.enableLimitSwitch(false);
    intakeMotor.restoreFactoryDefaults();
    intakeFollowerMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeFollowerMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSecondaryCurrentLimit(10);
    
    intakeMotor.setSmartCurrentLimit(10, 10);
    intakeFollowerMotor.follow(intakeMotor, false);
    intakeMotor.burnFlash();
    intakeFollowerMotor.burnFlash();
    setDefaultCommand(run(()->this.intake(0)));
  }

  @Log
  public boolean hitCurrentLimit() {
    return intakeMotor.getFault(FaultID.kOvercurrent);
  }

  @Log
  public boolean hitBeamBreak() {
    return m_beamBreak.isPressed();
  }

  public double getHandLength() {
    if (doubleSolenoid.get() == Value.kForward) {
      return Units.inchesToMeters(16);
    }
    else {
      return Units.inchesToMeters(9.4);
    }
  }

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
    intake(Constants.IntakeConstants.INTAKE_VOLTAGE * 2 * (isExtended() ? 1 : 2));
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

  public void setGamePiece(boolean isCube) {
    doubleSolenoid.set(isCube? Value.kForward : Value.kReverse);
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

  public boolean isExtended() {
    return doubleSolenoid.get() == Value.kForward;
  }

  /**
   * Runs the intake motor forward and then stops it
   * @return returns the runEnd Command
   */

   public Command setGamePieceC(BooleanSupplier isCube) {
    return runOnce(()->setGamePiece(isCube.getAsBoolean()));
   }
   public Command intakeC() {
    return runEnd(this::intake, this::stop);//.until(new Trigger(this::hitProxSensor));
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
   * Retracts the intake
   * @return returns the runOnce command
   */

  public Command retractC() {
    return runOnce(this::retract);
  }

  /**
   * extends the intake and runs the intake motor forward
   * @return returns the sequence Command
   */

  public Command extendAndIntakeC() {
    return sequence(extendC(), intakeC()).finallyDo((interrupted)-> stop());
  }

  /**
   * retracts the intake and runs the intake motor forward
   * @return returns the sequence Command
   */

  public Command retractAndIntakeC() {
    return sequence(retractC(), intakeC()).finallyDo((interrupted)-> stop());
  }

  /**
   * extends the intake and runs the intake motor in reverse
   * @return returns the sequence Command
   */

  public Command extendAndOuttakeC() {
    return sequence(extendC(), outtakeC()).finallyDo((interrupted)-> stop());
  }

  /**
   * retracts the intake and runs the intake motor in reverse
   * @return returns the sequence Command
   */

   public Command retractAndOuttakeC() {
    return sequence(retractC(), outtakeC()).finallyDo((interrupted)-> stop());
  }

  public Command intakeUntilBeamBreakC() {
    return intakeC().until(()->{
      return doubleSolenoid.get() == Value.kForward ? 
      cubeDebouncedBeamBreak.getAsBoolean() : coneDebouncedBeamBreak.getAsBoolean();})
      .andThen(intakeC().withTimeout(0.3));
    
  }

}
