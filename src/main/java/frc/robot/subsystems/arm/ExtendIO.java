package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.util.TimingTracer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public abstract class ExtendIO implements Loggable {

    public String configureLogName() {
        return "Extend";
    }
    protected final ProfiledPIDController m_extendController =
    new ProfiledPIDController(8,0,0,
        new Constraints(1.3, 1.5),
        0.02
    );

    protected final SimpleMotorFeedforward m_extendFeedforward
    = new SimpleMotorFeedforward(ARM_EXTEND_KS, ARM_EXTEND_KV, 0.01);

    protected DoubleSupplier m_angleSupplier = ()->0;

    @Log
    public final double minLength = MIN_ARM_LENGTH;

    public ExtendIO(Consumer<Runnable> addPeriodic) {
        m_extendController.reset(MIN_ARM_LENGTH);
    }

    public void setAngleSupplier(DoubleSupplier supplier) {
        m_angleSupplier = supplier;
    }

    /**
     * sets telescoping velocity of arm using feedforward
     * @param velocityMetersPerSecond desired telescoping velocity in meters per second
     */
     public void setVelocity(double velocityMetersPerSecond) {
        setVolts(
            m_extendFeedforward.calculate(
                getVelocity(),
                velocityMetersPerSecond, TimingTracer.getLoopTime()
            )
            + ARM_EXTEND_KG_VERTICAL * Math.sin(m_angleSupplier.getAsDouble())
        );
    }

    public void openLoopHold() {
        setVelocity(0);
    }

    /**
     * sets the desired arm length in meters
     * @param lengthMeters desired arm length in meters
     */

    public void setLength(double lengthMeters) {  
        if (isInTolerance()) {setVelocity(0); return;}
        setVelocity(
            m_extendController.calculate(getLength(), lengthMeters)
            + m_extendController.getSetpoint().velocity
        );
    }

    public void resetController() {
        m_extendController.reset(getLength());
    }

    public State getSetpoint() {
        return m_extendController.getSetpoint();
    }
    @Log
    public double getSetpointPosition() {
        return getSetpoint().position;
    }
    @Log
    public double getSetpointVelocity() {
        return getSetpoint().velocity;
    }
    public State getGoal() {
        return m_extendController.getGoal();
    }
    @Log
    public double getGoalPosition() {
        return getGoal().position;
    }
    @Log
    public double getGoalVelocity(){
        return getGoal().velocity;
    }
    @Log
    public abstract double getVolts();
    @Log
    public boolean isInTolerance() {
        return m_extendController.atSetpoint();
    }
    public abstract void setVolts(double volts);
    @Log
    public abstract double getLength();
    @Log
    public abstract double getVelocity();
    @Log
    public abstract boolean isHomed();
    public void onHome() {
        m_extendController.reset(MIN_ARM_LENGTH);
    }

}
