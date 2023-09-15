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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.sparkmax.SparkMax;

import static frc.robot.Constants.IntakeConstants.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeS extends SubsystemBase implements Loggable {
  private final SparkMax intakeMotor = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushless);
  @Log
  private boolean isCube = false;

  private final TimeOfFlight distanceSensor = new TimeOfFlight(Constants.IntakeConstants.INTAKE_TOF_CAN_ID);
  private Trigger cubeDebouncedBeamBreak = new Trigger(()->intakeMotor.getOutputCurrent() > 20).debounce(0.04);//.debounce(0.06);
  private Trigger coneDebouncedBeamBreak = new Trigger(()->intakeMotor.getOutputCurrent() > 20).debounce(0.1);

    /** Creates a new IntakeS. */
  public IntakeS() {

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSecondaryCurrentLimit(30);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    intakeMotor.setSmartCurrentLimit(30, 30);

    distanceSensor.setRangingMode(RangingMode.Short, 999);
    distanceSensor.setRangeOfInterest(9,9,11,11);
    setDefaultCommand(run(()->this.intake(isCube? 0.5 : 0)));
  }

  public Transform2d getConeCenterOffset() {
    
    double distanceToCone = getConeCenterOffsetDistance();
    if (Math.abs(distanceToCone) > 0.2) {
      return new Transform2d();
    }
    

    double offsetMeters = distanceToCone;
    return new Transform2d(new Translation2d(0, offsetMeters), new Rotation2d());
  }

  @Log
  public double getConeCenterOffsetDistance() {
    if (RobotBase.isSimulation()) {
      return 0;
    }
     return -(INTAKE_CENTERED_CONE_DISTANCE - distanceSensor.getRange()) / 1000.0;
  }
  //@Log
  public double getIntakeVolts() {
    return intakeMotor.getAppliedOutput() * 12;
  }

  //@Log
  public double getDistanceSensor() {
    return distanceSensor.getRange();
  }

  //@Log
  public double getRangeSigma() {
    return distanceSensor.getRangeSigma();
  }

  @Log
  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  // @Log
  // public boolean hitCurrentLimit() {
  //   return intakeMotor.getFault(FaultID.kOvercurrent);
  // }

  @Log
  public boolean hitBeamBreak() {
    return !isCube && (Math.abs(getConeCenterOffsetDistance())) < 0.16;
  }

  /**
   * Sets voltage of intake motor to the voltage parameter 
   * @param voltage the voltage supplied to intake motor
   */

  public void intake(double voltage) {
    intakeMotor.setVoltage(voltage  * (isCube() ? 1 : -1));
  }


  /**
   * Spins intake motor forward at specified voltage
   */

  public void intake() {
    intake((isCube? 1 : 1) * Constants.IntakeConstants.INTAKE_VOLTAGE);
  }

  public Command autoStagedIntakeC() {
    return runEnd(()->intake(Constants.IntakeConstants.INTAKE_VOLTAGE), this::stop);
  }

  /**
   * Spins intake motor in reverse at specified voltage
   */

  public void outtake() {
    intake(-Constants.IntakeConstants.INTAKE_VOLTAGE);
  }

  public void setGamePiece(boolean isCube) {
    this.isCube = isCube;
  }


  /**
   * Toggles the intake between extend and retract
   */

  public void toggle() {
    isCube = !isCube;
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

  public boolean isCube() {
    return isCube;
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

  public Command intakeUntilBeamBreakC() {
    return intakeUntilBeamBreakC(intakeC());
  }

  public Command intakeUntilBeamBreakC(Command intakeCommand) {
    return intakeCommand.until(()->{
      return isCube ? 
      cubeDebouncedBeamBreak.getAsBoolean() : coneDebouncedBeamBreak.getAsBoolean();})
      .andThen(intakeC().withTimeout(isCube ? 0.1 : 0.1));
    
  }



}
