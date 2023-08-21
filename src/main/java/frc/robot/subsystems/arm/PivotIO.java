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

public abstract class PivotIO implements Loggable {

    protected LinearSystem<N2, N1, N1> m_pivotPlant = LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getNEO(2), ARM_MOI_SHRUNK, 1.0 / ARM_ROTATIONS_PER_MOTOR_ROTATION);

    private State m_setpoint = new State();
    private State m_goal = new State();
    
    private final Constraints m_constraints = new Constraints(PIVOT_MAX_VELOCITY, PIVOT_MAX_ACCEL);
    private DCMotor m_pivotGearbox = DCMotor.getNEO(1);
    private LinearPlantInversionFeedforward<N2, N1, N1> m_pivotFeedForward = new LinearPlantInversionFeedforward<>(
            m_pivotPlant, 0.02);

    private LinearQuadraticRegulator<N2, N1, N1> m_pivotController = new LinearQuadraticRegulator<N2, N1, N1>(
            m_pivotPlant,
            VecBuilder.fill(0.01, 0.01),
            VecBuilder.fill(12),
            0.02);

    private DoubleSupplier m_extendLengthSupplier = () -> MIN_ARM_LENGTH;

    public PivotIO(Consumer<Runnable> addPeriodic) {
        addPeriodic.accept(this::updatePivotPlant);
        addPeriodic.accept(this::runPID);
    }

    public void setExtendLengthSupplier(DoubleSupplier extendLengthSupplier) {
        m_extendLengthSupplier = extendLengthSupplier;
    }

    public void resetController() {
        m_pivotController.reset();
        m_setpoint = new State(getContinuousRangeAngle(), 0);
        
    }
    public void resetGoal() {
        m_goal = new State(getContinuousRangeAngle(), 0);
    }

    @Log
    public boolean isInTolerance() {
        return Math.abs(getContinuousRangeAngle() - getGoalPosition()) < 0.05;
    }

    public State getSetpoint() {
        return m_setpoint;
    }

    @Log
    public double getSetpointPosition() {
        return getSetpoint().position;
    }

    public State getGoal() {
        return m_goal;
    }

    @Log
    public double getGoalPosition() {
        return getGoal().position;
    }

    @Log
    public abstract double getVolts();

    protected abstract void setVolts(double volts);

    /**
     * Sets voltage of pivot motor to the volts parameter
     * 
     * @param volts Desired voltage
     */

    public void setPivotVolts(double volts) {

        if (getContinuousRangeAngle() < MIN_ARM_ANGLE && volts < 0) {
            volts = 0;
        }
        if (getContinuousRangeAngle() > MAX_ARM_ANGLE && volts > 0) {
            volts = 0;
        }
        setVolts(MathUtil.clamp(volts, -12, 12));
    }

    /**
     * @return the angle of the pivot joint
     */

    // @Log(methodName = "getRadians")
    public Rotation2d getAngle() {
        double position = getContinuousRangeAngle();
        return Rotation2d.fromRadians(position);
    }

    @Log
    public abstract double getContinuousRangeAngle();

    public double continuousRangeAngleModulus(double targetAngle) {
        targetAngle = MathUtil.angleModulus(targetAngle);
        // now in range -pi to pi (-180 to 180)
        if (targetAngle <= -Math.PI / 2) {
            targetAngle += 2 * Math.PI;
        }
        // now in range -pi/2 to 3pi/2 (-90 to 270)
        return targetAngle;
    }

    private double getLength() {
        return m_extendLengthSupplier.getAsDouble();
    }

    /**
     * @return the MOI of the arm in Joules per kg^2
     */

    public double getPivotMOI() {

        // double minMOI = ARM_MOI_SHRUNK;
        // double maxMOI = A;
        // double result = minMOI;
        // double frac = (m_extendLengthSupplier.getAsDouble() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        // result += frac * frac * (maxMOI - minMOI);
        // return result;
        // var moiAboutCM = 1.0 / 12.0 * ARM_MASS_KILOS * m_extendLengthSupplier.getAsDouble()
        //         * m_extendLengthSupplier.getAsDouble();
        // return moiAboutCM;
        // TODO get this from held piece status and length
        return (1.0 / 3.0 * ARM_MASS_KILOS *
        (getLength()-Units.inchesToMeters(12.5)) *
        (getLength()-Units.inchesToMeters(12.5)) / 2.0) +
        (1.0 / 3.0 * ARM_MASS_KILOS * Units.inchesToMeters(12.5) *
        Units.inchesToMeters(12.5) / 2.0);
    }

    @Log
    public double getPivotCGRadius() {
        double minCG = 0;
        double maxCG = Units.inchesToMeters(8);

        double result = minCG;
        double frac = (m_extendLengthSupplier.getAsDouble() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        result += frac * (maxCG - minCG);
        return result;
    }

    public void resetPivot() {
        resetController();
    }

    /**
     * updates the pivot plant
     */

    public void updatePivotPlant() {
        m_pivotPlant.getA().set(1, 1,
                -1.0 / ARM_ROTATIONS_PER_MOTOR_ROTATION * 1.0 / ARM_ROTATIONS_PER_MOTOR_ROTATION
                        * m_pivotGearbox.KtNMPerAmp
                        / (m_pivotGearbox.KvRadPerSecPerVolt * m_pivotGearbox.rOhms * getPivotMOI()));
        m_pivotPlant.getB().set(1, 0,
                1.0 / ARM_ROTATIONS_PER_MOTOR_ROTATION * m_pivotGearbox.KtNMPerAmp
                        / (m_pivotGearbox.rOhms * getPivotMOI()));
        m_pivotController = new LinearQuadraticRegulator<N2, N1, N1>(
            m_pivotPlant, VecBuilder.fill(0.05, 0.05), VecBuilder.fill(12), 0.02);
    }

    /**
     * sets pivot motor to desired velocity in radians per second
     * 
     * @param velocityRadPerSec desired velocity in radians per second
     */

    protected void setPivotVelocity(double velocityRadPerSec) {
        var currentVoltageAdd = (getPivotkG() * getAngle().getCos())
                + PIVOT_KS * Math.signum(velocityRadPerSec);
        setPivotVolts(m_pivotFeedForward.calculate(
                VecBuilder.fill(0, velocityRadPerSec))
                .get(0, 0)
                + currentVoltageAdd);
        // SmartDashboard.putNumber("armCommandVelocity", velocityRadPerSec);
    }

    /**
     * sets pivot joint to desired angle in radians
     * 
     * @param targetAngle desired angle in radians
     */

    private void runPID() {
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        var nextSetpoint = profile.calculate(0.04);
        var currentVoltageAdd = (getPivotkG() * getAngle().getCos())
                + PIVOT_KS * Math.signum(m_setpoint.velocity);
        setVolts(
                m_pivotController.calculate(
                        VecBuilder.fill(getContinuousRangeAngle(), 0),
                        VecBuilder.fill(m_setpoint.position, 0)).get(0, 0)
                        + currentVoltageAdd
                        + m_pivotFeedForward.calculate(
                                VecBuilder.fill(0, m_setpoint.velocity),
                                VecBuilder.fill(0, nextSetpoint.velocity))
                                .get(0, 0) * 1.21);
    }

    public void setAngle(double targetAngle) {
        // We need to convert this to -90 to 270.
        // We don't want continuous input, but we need the rollover point to be outside
        // our range of motion.

        targetAngle = continuousRangeAngleModulus(targetAngle);
        m_goal = new State(targetAngle, 0);
        
    }

    /**
     * calculates voltage required to counteract the force of gravity on the arm
     * by interpolating between minimum and maximum arm lengths
     * 
     * @return returns the required voltage needed
     *         to hold arm horizontal at current extension length
     */

    @Log
    protected double getPivotkG() {
        return getkG(getContinuousRangeAngle());
        // double minkG = ARM_PIVOT_KG_MIN_EXTEND;
        // double maxkG = ARM_PIVOT_KG_MAX_EXTEND;
        // double result = minkG;
        // double frac = (m_extendLengthSupplier.getAsDouble() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        // result += frac * (maxkG - minkG);
        //return result;
    }

    public double getkG(double angle) {
        DCMotor m_gearbox = DCMotor.getNEO(2);
        double voltsDrawnByGravity = ARM_MASS_KILOS
        * getPivotCGRadius()
        * -9.8
        * Math.cos(angle)
        * (m_gearbox.rOhms)
        / (m_gearbox.KtNMPerAmp * 1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION );
        return -voltsDrawnByGravity;
         
      }

    public String configureLogName() {
        return "Pivot";
    }
}
