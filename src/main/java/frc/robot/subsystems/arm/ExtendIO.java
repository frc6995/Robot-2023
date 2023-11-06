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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.TimingTracer;
import autolog.Logged;
import autolog.AutoLog.BothLog;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

import java.sql.Driver;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

public abstract class ExtendIO implements Logged {

    public String getPath() {
        return "Extend";
    }
    // protected final ProfiledPIDController m_extendController =
    // new ProfiledPIDController(8,0,0,
    //     new Constraints(1.3, 1.5),
    //     0.02
    // );
    protected final LinearSystem<N2, N1, N1> m_extendPlant =
    //LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), 3.9159, EXTEND_DRUM_RADIUS, EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION);
    LinearSystemId.identifyPositionSystem(ARM_EXTEND_KV, ARM_EXTEND_KA);
    protected final PIDController m_extendController = new PIDController(5.625 * 5, 0, 0.71);

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints m_constraints = new Constraints(2.0, 4.5);

    protected final SimpleMotorFeedforward m_extendFeedforward = new SimpleMotorFeedforward(ARM_EXTEND_KS, ARM_EXTEND_KV, ARM_EXTEND_KA);

    protected DoubleSupplier m_angleSupplier = ()->0;

    
    public final double minLength = MIN_ARM_LENGTH;

    public ExtendIO(Consumer<Runnable> addPeriodic) {
        //m_extendController.latencyCompensate(m_extendPlant, 0.02, 0.01);
        m_extendController.reset();
        m_setpoint = new State(MIN_ARM_LENGTH, 0);
        m_goal = new State(MIN_ARM_LENGTH, 0);
        addPeriodic.accept(this::runPID);
        addPeriodic.accept(()->{
            if (DriverStation.isDisabled()) {
                resetController();
                resetGoal();
            }
        });

    }

    public void onEnabled(){
        resetController();
        resetGoal();
    }
    @BothLog
    public double getCurrent() {return 0;}
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
        m_setpoint = profile.calculate(0.02);
        State nextSetpoint = profile.calculate(0.04);
        
        setPIDFF(m_setpoint.position,
             (m_extendFeedforward.calculate(m_setpoint.velocity, nextSetpoint.velocity, 0.02)

            ) +
            ARM_EXTEND_KG_VERTICAL * Math.sin(m_angleSupplier.getAsDouble())
        );
    }

    protected void setPIDFF(double length, double ffVolts) {
        setVolts(
            m_extendController.calculate(getLength(), length) + ffVolts);
    }

    public double getPeriod() {
        return 0.02;
    }

    public void resetController() {
        m_extendController.reset();
        m_setpoint = new State(getLength(), getVelocity() );
    }
    public void resetGoal() {
        m_goal = new State(getLength(), 0 );
    }

    public State getSetpoint() {
        return m_setpoint;
    }
    @BothLog
    public double getSetpointPosition() {
        return getSetpoint().position;
    }
    public double getSetpointVelocity() {
        return getSetpoint().velocity;
    }
    public State getGoal() {
        return m_goal;
    }
    @BothLog
    public double getGoalPosition() {
        return getGoal().position;
    }
    @BothLog
    public double getGoalVelocity(){
        return getGoal().velocity;
    }
    @BothLog
    public double getKG() {
        return ARM_EXTEND_KG_VERTICAL * Math.sin(m_angleSupplier.getAsDouble());
    }
    @BothLog
    public abstract double getVolts();
    @BothLog
    public boolean isInTolerance() {
        return Math.abs(getLength() - getGoalPosition()) < Units.inchesToMeters(0.5);
    }
    public abstract void setVolts(double volts);
    @BothLog
    public abstract double getLength();
    @BothLog
    public abstract double getVelocity();
    @BothLog
    public abstract boolean isHomed();
    public abstract void setIdleMode(IdleMode mode);
    public void onHome() {
        m_extendController.reset();
        //m_extendFeedforward.reset(VecBuilder.fill(MIN_ARM_LENGTH, 0));
        m_setpoint = new State(MIN_ARM_LENGTH, 0);
    }
    


}
