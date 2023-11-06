package frc.robot.util.sim;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Robot;

public class SparkMaxAbsoluteEncoderWrapper {
    private final CANSparkMax sparkMax;
    private final AbsoluteEncoder sparkMaxEncoder;
    private double lastPosition = 1.0;
    private double simPosition = 1.50;
    private double simVelocity = 0.0;
  
    /**
     * Creates a new SparkMaxDerivedVelocityController using a default set of parameters.
     */
    public SparkMaxAbsoluteEncoderWrapper(CANSparkMax sparkMax, double offset) {
        this.sparkMax = sparkMax;
      this.sparkMaxEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      sparkMaxEncoder.setZeroOffset(offset);
    }

  
    /**
     * Returns the current position in rotations.
     */
    public double getPosition() {
        if(Robot.isReal()) {
            double position = sparkMaxEncoder.getPosition();
            if (sparkMax.getLastError().equals(REVLibError.kTimeout)) {
                return lastPosition;
            }
            lastPosition = position;
            return position;
        }
        else {
            return simPosition;
        }
    }
  
    /**
     * Returns the current velocity in rotations/minute.
     */
    public synchronized double getVelocity() {
        if(Robot.isReal()) {
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
