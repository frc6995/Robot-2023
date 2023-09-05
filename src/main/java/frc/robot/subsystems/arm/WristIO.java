package frc.robot.subsystems.arm;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

public abstract class WristIO implements Loggable {

    protected final double wristMOI = HAND_MASS_KILOS * HAND_LENGTH * HAND_LENGTH / 2.5;

    protected final LinearSystem<N2, N1, N1> m_wristPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(1), wristMOI, 1.0/WRIST_ROTATIONS_PER_MOTOR_ROTATION);
    private State m_setpoint = new State();
    private State m_goal = new State();
    @Log
    protected boolean isHoming= false;
    @Log
    protected boolean hasHomed = false;
    protected final LinearPlantInversionFeedforward<N2, N1, N1> m_wristFeedforward
        = new LinearPlantInversionFeedforward<>(m_wristPlant, 0.02);
    protected final Constraints m_constraints = new Constraints(3*Math.PI, 6*Math.PI);
    
    protected final LinearQuadraticRegulator<N2, N1, N1> m_wristController = 
    new LinearQuadraticRegulator<>(m_wristPlant, VecBuilder.fill(0.001, 0.001), VecBuilder.fill(12), 0.02);

    protected DoubleSupplier m_pivotAngleSupplier = ()->0;
    protected Alert m_wristNotHomedAlert = new Alert("Wrist", "Not Homed", AlertType.ERROR);

    public WristIO(Consumer<Runnable> addPeriodic) {
        m_goal = new State(STOW_POSITION.wristRadians, 0);
        m_setpoint = new State(STOW_POSITION.wristRadians, 0);
        addPeriodic.accept(this::runPID);
        addPeriodic.accept(()->{
            m_wristNotHomedAlert.set(!hasHomed);
            if (isHoming) {LightStripS.getInstance().requestState(States.Climbing);};
    });
        addPeriodic.accept(()->{
            if (DriverStation.isDisabled()) {
                resetController();
                resetGoal();
            }
        });
        new Trigger(()->isHoming && getHomed()).debounce(0.06).onTrue(Commands.runOnce(this::endHome));
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

    public abstract void resetState(double position);
    public void startHome() {
        isHoming = true;
        resetState(WRIST_MIN_ANGLE - (WRIST_MAX_ANGLE - WRIST_MIN_ANGLE));
    }

    public void endHome() {
        isHoming = false;
        hasHomed = true;
        resetState(WRIST_MAX_ANGLE);
        resetController();
        resetGoal();
        m_setpoint=new State(WRIST_MAX_ANGLE, 0);
    }

    private void runPID() {
        var profile = new TrapezoidProfile(getConstraints(), m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        var nextSetpoint = profile.calculate(0.04);
        setVolts(
            m_wristController.calculate(
                        VecBuilder.fill(getAngle(), getVelocity()),
                        VecBuilder.fill(m_setpoint.position, m_setpoint.velocity)).get(0, 0)
                        + getWristkG()
                        + m_wristFeedforward.calculate(
                                VecBuilder.fill(0, m_setpoint.velocity),
                                VecBuilder.fill(0, nextSetpoint.velocity))
                                .get(0, 0));
    }

    public void openLoopHold() {
        setVolts(getWristkG());
    }


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


    public String configureLogName() {
        return "Wrist";
    }

    @Log
    public boolean isInTolerance() {
        return Math.abs(getAngle() - getGoalPosition()) < 0.05;
    }

    public abstract void setIdleMode(IdleMode mode);
    public abstract double getVelocity();
    protected abstract void setVoltsInternal(double volts);
    /**
     * 0 is straight out from the arm, parallel to it.
     * @return the wrist angle from -pi radians to pi radians
     */
    @Log
    public abstract double getAngle();
    @Log
    public abstract double getVolts();
    @Log
    public abstract double getCurrent();
    @Log
    public abstract boolean getHomed();
    public Constraints getConstraints() {
        return m_constraints;
    };
}
