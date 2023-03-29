// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeS extends SubsystemBase implements Loggable {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax intakeFollowerMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_FOLLOWER_CAN_ID, MotorType.kBrushless);
  private final SparkMaxLimitSwitch m_beamBreak = intakeFollowerMotor.getReverseLimitSwitch(Type.kNormallyClosed);
  @Log
  private boolean isExtended = false;
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
    Constants.IntakeConstants.INTAKE_EXTEND, Constants.IntakeConstants.INTAKE_RETRACT);

  private final TimeOfFlight distanceSensor = new TimeOfFlight(Constants.IntakeConstants.INTAKE_TOF_CAN_ID);
  private Trigger cubeDebouncedBeamBreak = new Trigger(this::hitBeamBreak);//.debounce(0.06);
  private Trigger coneDebouncedBeamBreak = new Trigger(this::hitBeamBreak);//.debounce(0.0);
    /** Creates a new IntakeS. */
  public IntakeS() {

    intakeMotor.restoreFactoryDefaults();
    intakeFollowerMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeFollowerMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSecondaryCurrentLimit(15);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 45);
    intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    m_beamBreak.enableLimitSwitch(false);
    
    intakeMotor.setSmartCurrentLimit(10, 10);
    intakeFollowerMotor.follow(intakeMotor, false);
    intakeMotor.burnFlash();
    intakeFollowerMotor.burnFlash();

    distanceSensor.setRangingMode(RangingMode.Short, 1000);
    distanceSensor.setRangeOfInterest(9,9,11,11);
    setDefaultCommand(run(()->this.intake(0)));
  }

  public Transform2d getConeCenterOffset() {
    
    double distanceToCone = getConeCenterOffsetDistance();
    if (Math.abs(distanceToCone) > 0.1) {
      return new Transform2d();
    }
    

    double offsetMeters = distanceToCone;
    return new Transform2d(new Translation2d(0, offsetMeters), new Rotation2d());
  }

  @Log
  public double getConeCenterOffsetDistance() {
     return (IntakeConstants.INTAKE_CENTERED_CONE_DISTANCE - distanceSensor.getRange()) / 1000.0;
  }
  @Log
  public double getIntakeVolts() {
    return intakeMotor.getAppliedOutput() * 12;
  }

  @Log
  public double getDistanceSensor() {
    return distanceSensor.getRange();
  }

  @Log
  public double getRangeSigma() {
    return distanceSensor.getRangeSigma();
  }

  @Log
  public boolean hitCurrentLimit() {
    return intakeMotor.getFault(FaultID.kOvercurrent);
  }

  @Log
  public boolean hitBeamBreak() {
    return m_beamBreak.isPressed() || (Math.abs(getConeCenterOffsetDistance()) > 0.1);
  }

  public double getHandLength() {
    if (isExtended) {
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
    intake(Constants.IntakeConstants.INTAKE_VOLTAGE * (isExtended() ? 1.5 : 3));
  }

  public Command autoStagedIntakeC() {
    return runEnd(()->intake(Constants.IntakeConstants.INTAKE_VOLTAGE * 3 * 2.9/3.4), this::stop);
  }

  /**
   * Spins intake motor in reverse at specified voltage
   */

  public void outtake() {
    intake(-Constants.IntakeConstants.INTAKE_VOLTAGE * (isExtended() ? 2 : 1));
  }

  /**
   * Extends the intake
   */

  public void extend() {
    doubleSolenoid.set(Value.kForward);
    isExtended = true;
  }

  /**
   * Retracts the intake
   */

  public void retract() {
    doubleSolenoid.set(Value.kReverse);
    isExtended = false;
  }

  public void setGamePiece(boolean isCube) {
    doubleSolenoid.set(isCube? Value.kForward : Value.kReverse);
    isExtended = isCube;
  }

  @Log
  public boolean solenoidState() {
    return doubleSolenoid.get() == Value.kReverse;
  }

  /**
   * Toggles the intake between extend and retract
   */

  public void toggle() {
    if (doubleSolenoid.get() == Value.kOff) {
      extend();
      isExtended = true;
    }
    else {
      doubleSolenoid.toggle();
      isExtended = !isExtended;
    }

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
    return isExtended;
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
    return intakeUntilBeamBreakC(intakeC());
  }

  public Command intakeUntilBeamBreakC(Command intakeCommand) {
    return intakeCommand.until(()->{
      return isExtended ? 
      cubeDebouncedBeamBreak.getAsBoolean() : coneDebouncedBeamBreak.getAsBoolean();})
      .andThen(intakeC().withTimeout(isExtended ? 0.1 : 0.1));
    
  }



}
