package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
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
    // protected final ProfiledPIDController m_extendController =
    // new ProfiledPIDController(8,0,0,
    //     new Constraints(1.3, 1.5),
    //     0.02
    // );
    protected final LinearSystem<N2, N1, N1> m_extendPlant =
    LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), 3.9159, EXTEND_DRUM_RADIUS, EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION);
    // LinearSystemId.identifyPositionSystem(
    //     ARM_EXTEND_KV,
    //     0.01);
    protected final LinearQuadraticRegulator<N2, N1, N1> m_extendController = new LinearQuadraticRegulator<N2, N1, N1>(
        m_extendPlant,
        VecBuilder.fill(0.001, 0.001),
        VecBuilder.fill(12),
        0.02
    );
    private double m_minimumInput;
    private double m_maximumInput;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints m_constraints = new Constraints(1.3 * 1.3, 3.9);
    protected final LinearPlantInversionFeedforward<N2, N1, N1> m_extendFeedforward
    = new LinearPlantInversionFeedforward<>(m_extendPlant, 0.02);

    protected DoubleSupplier m_angleSupplier = ()->0;

    @Log
    public final double minLength = MIN_ARM_LENGTH;

    public ExtendIO(Consumer<Runnable> addPeriodic) {
        m_extendController.reset();
        m_setpoint = new State(MIN_ARM_LENGTH, 0);
        m_goal = new State(MIN_ARM_LENGTH, 0);
        addPeriodic.accept(this::runPID);

    }

    public void setAngleSupplier(DoubleSupplier supplier) {
        m_angleSupplier = supplier;
    }

    /**
     * sets the desired arm length in meters
     * @param lengthMeters desired arm length in meters
     */

    public void setLength(double lengthMeters) {  
        //if (isInTolerance()) {setVelocity(0); return;}
        m_goal = new State(lengthMeters, 0);

    }
    private void runPID() {
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(TimingTracer.getLoopTime());
        State nextSetpoint = profile.calculate(getPeriod() * 2.0);
        setVolts(
            m_extendController.calculate(
                VecBuilder.fill(getLength(), getVelocity()),
                VecBuilder.fill(m_setpoint.position, m_setpoint.velocity)
            ).get(0, 0) +
            (m_extendFeedforward.calculate(
                VecBuilder.fill(m_setpoint.position, m_setpoint.velocity), 
                VecBuilder.fill(nextSetpoint.position, nextSetpoint.velocity)
                ).get(0,0) * 1.245
            )
            + ARM_EXTEND_KG_VERTICAL * Math.sin(m_angleSupplier.getAsDouble())
        );
    }

    public double getPeriod() {
        return 0.02;
    }

    public void resetController() {
        m_extendController.reset();
        m_setpoint = new State(getLength(), 0 );
    }
    public void resetGoal() {
        m_goal = new State(getLength(), 0 );
    }

    public State getSetpoint() {
        return m_setpoint;
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
        return m_goal;
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
        return Math.abs(getLength() - getGoalPosition()) < Units.inchesToMeters(0.5);
    }
    public abstract void setVolts(double volts);
    @Log
    public abstract double getLength();
    @Log
    public abstract double getVelocity();
    @Log
    public abstract boolean isHomed();
    public void onHome() {
        m_extendController.reset();
        m_setpoint = new State(MIN_ARM_LENGTH, 0);
    }

}
