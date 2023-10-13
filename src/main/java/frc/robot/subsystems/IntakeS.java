// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import autolog.Logged;
import autolog.AutoLog.BothLog;
import autolog.AutoLog.BothLog;
import edu.wpi.first.math.filter.LinearFilter;
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
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.sparkmax.SparkMax;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeS extends SubsystemBase implements Logged {
  private final SparkMax intakeMotor = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushless);
  /**
   * Set driver mode on the USB camera streamed through PhotonVision
   */
  PhotonCamera handCam = new PhotonCamera("HD_USB_Camera");
  private final TimeOfFlight distanceSensor = new TimeOfFlight(Constants.IntakeConstants.INTAKE_TOF_CAN_ID);
  private Trigger cubeDebouncedBeamBreak = new Trigger(()->getCurrent() > 20);//.debounce(0.06);
  private Trigger coneDebouncedBeamBreak = new Trigger(()->getCurrent() > 20);
  private LinearFilter currentAverage = LinearFilter.movingAverage(10);
  private double current = 0;

    /** Creates a new IntakeS. */
  public IntakeS() {

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSecondaryCurrentLimit(700);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 45);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    intakeMotor.setSmartCurrentLimit(60, 60);

    distanceSensor.setRangingMode(RangingMode.Short, 999);
    distanceSensor.setRangeOfInterest(9,9,11,11);
    setDefaultCommand(run(()->intakeMotor.setVoltage(0)));
  }

  public Transform2d getConeCenterOffset() {
    
    double distanceToCone = getConeCenterOffsetDistance();
    if (!hitBeamBreak()) {
      return new Transform2d();
    }
    

    double offsetMeters = distanceToCone;
    return new Transform2d(new Translation2d(0, offsetMeters), new Rotation2d());
  }

  @BothLog
  public double getConeCenterOffsetDistance() {
    if (RobotBase.isSimulation()) {
      return 0;
    }
     return -(INTAKE_CENTERED_CONE_DISTANCE - distanceSensor.getRange()) / 1000.0;
  }

  public boolean hitBeamBreak() {
    return getConeCenterOffsetDistance() > -0.2 && getConeCenterOffsetDistance() < 0.15;
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

  @BothLog
  public double getCurrent() {
    return current;
  }
  public double getUnfilteredCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  // @Log
  // public boolean hitCurrentLimit() {
  //   return intakeMotor.getFault(FaultID.kOvercurrent);
  // }
  public void intakeCube() {
    intakeMotor.setVoltage(Constants.IntakeConstants.INTAKE_VOLTAGE);
  }
  public void intakeCube(double volts) {
    intakeMotor.setVoltage(volts);
  }
  public void outtakeCone(double volts) {
    intakeCube(volts);
  }
  public void outtakeCube(double volts) {
    intakeCone(volts);
  }
  public void intakeCone() {
    intakeMotor.setVoltage(-Constants.IntakeConstants.INTAKE_VOLTAGE);
  }
  public void intakeCone(double volts) {
    intakeMotor.setVoltage(-volts);
  }
  public void outtakeCube() {
    intakeMotor.setVoltage(-0.4 * Constants.IntakeConstants.INTAKE_VOLTAGE);
  }
  public void outtakeCubeSlow() {
    intakeMotor.setVoltage(-0.4 * Constants.IntakeConstants.INTAKE_VOLTAGE);
  }
  public void outtakeCone() {
    intakeMotor.setVoltage(0.6 * Constants.IntakeConstants.INTAKE_VOLTAGE);
  }

  public void intake(boolean isCube) {
    if (isCube) {
      intakeCube();
    } else {
      intakeCone();
    }
  }

  public void outtake(boolean isCube) {
    if (isCube) {
      outtakeCube();
    } else {
      outtakeCone();
    }
  }

  public void outtakeSlow(boolean isCube) {
    if (isCube) {
      intakeMotor.setVoltage(-0.2 * Constants.IntakeConstants.INTAKE_VOLTAGE);
    } else {
      outtakeCone();
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
    current = currentAverage.calculate(intakeMotor.getOutputCurrent());
  }

  public Command intakeC(BooleanSupplier isCube) {
    return runEnd(()->this.intake(isCube.getAsBoolean()), this::stop);//.until(new Trigger(this::hitProxSensor));
  }
  
  

  /**
   * Runs the intake motor in reverse and then stops it
   * @return returns the runEnd Command
   */

  public Command outtakeC(BooleanSupplier isCube) {
    return runEnd(()->this.outtake(isCube.getAsBoolean()), this::stop);
  }

  public Command outtakeC(BooleanSupplier isCube, BooleanSupplier slowCube) {
    return Commands.either(
      runEnd(()->this.outtakeSlow(isCube.getAsBoolean()), this::stop),
      runEnd(()->this.outtake(isCube.getAsBoolean()), this::stop),
      slowCube)
      ;
  }

  public Command intakeUntilBeamBreakC(boolean isCube) {
    return Commands.sequence(
      run(()->this.intake(isCube)).withTimeout(0.5),
      run(()->this.intake(isCube)).until(()->{
        return isCube ? 
        cubeDebouncedBeamBreak.getAsBoolean() : coneDebouncedBeamBreak.getAsBoolean();}),
      intakeC(()->isCube).withTimeout(isCube ? 0.1 : 0.0));
    }
  public boolean acquiredCube() {
    return cubeDebouncedBeamBreak.getAsBoolean();
  }
  public boolean acquiredCone() {
    return coneDebouncedBeamBreak.getAsBoolean();
  }
  public boolean acquiredPiece(boolean isCube) {
    if (isCube) {
      return acquiredCube();
    } else {
      return acquiredCone();
    }
  }

  public void setHandCamFlipped(boolean flipped) {
    handCam.setPipelineIndex(1);
  }
}
