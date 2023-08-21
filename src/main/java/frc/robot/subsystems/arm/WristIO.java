package frc.robot.subsystems.arm;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

public abstract class WristIO implements Loggable {

    protected final double wristMOI = HAND_MASS_KILOS * HAND_LENGTH * HAND_LENGTH / 3.0;

    protected final LinearSystem<N2, N1, N1> m_wristPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(1), wristMOI, 1.0/WRIST_ROTATIONS_PER_MOTOR_ROTATION);
    private State m_setpoint = new State();
    private State m_goal = new State();

    protected final LinearPlantInversionFeedforward<N2, N1, N1> m_wristFeedforward
        = new LinearPlantInversionFeedforward<>(m_wristPlant, 0.02);
        private final Constraints m_constraints = new Constraints(2, 6);
    
    protected final LinearQuadraticRegulator<N2, N1, N1> m_wristController = 
    new LinearQuadraticRegulator<>(m_wristPlant, VecBuilder.fill(0.01, 0.01), VecBuilder.fill(12), 0.02);

    protected DoubleSupplier m_pivotAngleSupplier = ()->0;
    public WristIO(Consumer<Runnable> addPeriodic) {
        m_goal = new State(STOW_POSITION.wristRadians, 0);
        m_setpoint = new State(STOW_POSITION.wristRadians, 0);
        addPeriodic.accept(this::runPID);
    }

    public void setPivotAngleSupplier (DoubleSupplier pivotAngleSupplier) {
        m_pivotAngleSupplier = pivotAngleSupplier;
    }
    @Log
    protected double getWristkG() {
        return WRIST_KG * Math.cos(m_pivotAngleSupplier.getAsDouble() + getAngle());
    }

    private void setVelocity(double velocityRadPerSec) {
        setVolts(
            m_wristFeedforward.calculate(VecBuilder.fill(0, velocityRadPerSec)).get(0, 0)
            + getWristkG()
        );
    }

    public void resetController() {
        m_wristController.reset();
        double angle = getAngle();
        m_setpoint = new State(angle, 0);
    }

    public void resetGoal() {
        m_goal = new State(getAngle(), 0);
    }

    public void setAngle(double angleRad) {
        angleRad = MathUtil.angleModulus(angleRad);
        m_goal = new State(angleRad, 0);
    }

    private void runPID() {
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        var nextSetpoint = profile.calculate(0.04);
        setVolts(
            m_wristController.calculate(
                        VecBuilder.fill(getAngle(), 0),
                        VecBuilder.fill(m_setpoint.position, 0)).get(0, 0)
                        + getWristkG()
                        + m_wristFeedforward.calculate(
                                VecBuilder.fill(0, m_setpoint.velocity),
                                VecBuilder.fill(0, nextSetpoint.velocity))
                                .get(0, 0));
    }

    public void openLoopHold() {
        setVolts(getWristkG());
    }
    /**
     * 0 is straight out from the arm, parallel to it.
     * @return the wrist angle from -pi radians to pi radians
     */
    @Log
    public abstract double getAngle();
    @Log
    public abstract double getVolts();
    private void setVolts(double volts) {
        if (getAngle() > WRIST_MAX_ANGLE && volts > 0) {
            volts = 0;
        }
        if (getAngle() < WRIST_MIN_ANGLE && volts < 0) {
            volts = 0;
        }
        setVoltsInternal(volts);
    }
    
    public State getSetpoint() {
        return m_setpoint;
    }
    @Log
    public double getSetpointPosition(){
        return getSetpoint().position;
    }
    public State getGoal() {
        return m_goal;
    }
    @Log
    public double getGoalPosition() {
        return getGoal().position;
    }
    protected abstract void setVoltsInternal(double volts);

    public String configureLogName() {
        return "Wrist";
    }

    @Log
    public boolean isInTolerance() {
        return Math.abs(getAngle() - getGoalPosition()) < 0.05;
    }
}
