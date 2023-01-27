package frc.robot.util.sim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;

public class SparkMaxAbsoluteEncoderWrapper {
 
    private final AbsoluteEncoder sparkMaxEncoder;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;
  
    /**
     * Creates a new SparkMaxDerivedVelocityController using a default set of parameters.
     */
    public SparkMaxAbsoluteEncoderWrapper(CANSparkMax sparkMax, double offset) {
      this.sparkMaxEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      sparkMaxEncoder.setZeroOffset(offset);
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

    /**
     * Sets the sim position to the position parameter
     * @param position the value returned in the simulator from getPosition
     */

    public void setSimPosition(double position) {
        simPosition = position;
    }

    /**
     * Sets the sim velocity to the velocity parameter
     * @param velocity the value returned in the simulator from getVelocity
     */

    public void setSimVelocity(double velocity) {
        simVelocity = velocity;
    }

  }
