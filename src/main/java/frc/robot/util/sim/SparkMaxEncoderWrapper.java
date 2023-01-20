package frc.robot.util.sim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;

public class SparkMaxEncoderWrapper {
 
    private final RelativeEncoder sparkMaxEncoder;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;
  
    /**
     * Creates a new SparkMaxDerivedVelocityController using a default set of parameters.
     */
    public SparkMaxEncoderWrapper(CANSparkMax sparkMax) {
      this.sparkMaxEncoder = sparkMax.getEncoder();
    }

  
    /**
     * Returns the current position in rotations.
     */
    public double getPosition() {
        if(RobotBase.isReal()) {
            return sparkMaxEncoder.getPosition();
        }
        else {
            return simPosition;
        }
      
    }
  
    /**
     * Returns the current velocity in rotations/minute.
     */
    public synchronized double getVelocity() {
        if(RobotBase.isReal()) {
            return sparkMaxEncoder.getVelocity();
        } else {
            return simVelocity;
        }
    }

    public synchronized void setPosition(double position) {
        // we still want the encoder to report in motor shaft rotations, so divide by conversion factor.
        sparkMaxEncoder.setPosition(position);
        setSimPosition(position);
    }

    public synchronized void setSimPosition(double position) {
        simPosition = position;
    }

    public synchronized void setSimVelocity(double velocity) {
        simVelocity = velocity;
    }

    
  }
