package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.DriveConstants.*;

public abstract class ModuleIO implements Loggable {

    // steering trapezoid profile
    protected State m_steerGoal = new State();
    protected State m_steerSetpoint = new State();
    protected TrapezoidProfile.Constraints m_steeringConstraints = new TrapezoidProfile.Constraints(
            STEER_MAX_SPEED_RAD_PER_SEC,
            STEER_MAX_ACCEL_RAD_PER_SEC_SQ);

    private final PIDController m_steerPIDController;
    // logging position error because it's actually the "process variable", vs its
    // derivative
    private final PIDController m_drivePIDController;
    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            Robot.isReal() ? DRIVE_FF_CONST[0] : 0,
            DRIVE_FF_CONST[1],
            DRIVE_FF_CONST[2]);
    private final SimpleMotorFeedforward m_steerFeedForward = new SimpleMotorFeedforward(
        0, STEER_KV, 0.001);
    
    private String m_loggingName;
    // steering PID controller
    // drive PID controller
    protected final ModuleConstants m_moduleConstants;
    private final Alert m_pinionSlipAlert;

    public ModuleIO(ModuleConstants moduleConstants) {
        m_moduleConstants = moduleConstants;
        m_loggingName = moduleConstants.name + "-[" + moduleConstants.driveMotorID + ','
                + moduleConstants.rotationMotorID + ']';
        m_pinionSlipAlert = new Alert(moduleConstants.name, "Check pinion slip", AlertType.ERROR);
        m_steerPIDController = new PIDController(
                10 / (Math.PI/2), 0.0, STEER_D);
        // Tell the PID controller that it can move across the -pi to pi rollover point.
        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // For a velocity controller we just use P
        // (and feedforward, which is handled in #setDesiredStateClosedLoop)
        m_drivePIDController = new PIDController(
                DRIVE_P, 0,
                DRIVE_D);
    }

    public void updateAlerts() {
        m_pinionSlipAlert.set(Math.abs(getPinionSlip()) > Units.degreesToRadians(10));
    }

    public String configureLogName() {
        System.out.println(m_loggingName);
        return m_loggingName;
    }

    public abstract void setDriveVoltage(double driveVolts);

    public abstract void setRotationVoltage(double rotationVolts);

    @Log
    public abstract double getDriveDistance();

    @Log
    public abstract double getDriveVelocity();

    @Log
    /**
     * Returns the angle of the module from pi to -pi
     * @return
     */
    public abstract double getAngle();

    @Log
    public abstract double getRelativeAngle();

    @Log
    public abstract double getDriveVoltage();
    @Log
    public abstract double getSteerVoltage();

    @Log 
    public double getSteerSetpoint() {
        return m_steerSetpoint.position;
    }
    @Log
    public double getSteerGoal() {
        return m_steerGoal.position;
    }
    @Log
    public double getDriveSetpoint() {
        return m_drivePIDController.getSetpoint();
    }
    @Log
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state,new Rotation2d(getAngle()));
        double prevVelSetpoint = m_drivePIDController.getSetpoint();
        if (m_moduleConstants.name.contains("F")) {state.speedMetersPerSecond = -state.speedMetersPerSecond;}
        setDriveVoltage(
                m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond)
                        + m_driveFeedForward.calculate(prevVelSetpoint, state.speedMetersPerSecond, 0.02));

        
        double prevSteerGoal = m_steerGoal.position;
        m_steerGoal = new State(state.angle.getRadians(), 0);
                // Get error which is the smallest distance between goal and measurement
        double errorBound = Math.PI;
        double goalMinDistance =
            MathUtil.inputModulus(m_steerGoal.position - getAngle(), -errorBound, errorBound);
        double setpointMinDistance =
            MathUtil.inputModulus(m_steerSetpoint.position - getAngle(), -errorBound, errorBound);

        // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs to
        // be offset from the measurement by the input range modulus; they don't need to be equal.
        m_steerGoal.position = goalMinDistance + getAngle();
        m_steerSetpoint.position = setpointMinDistance + getAngle();


        m_steerGoal = new State(m_steerGoal.position, new Rotation2d(m_steerGoal.position).minus(new Rotation2d(prevSteerGoal)).getRadians() / 0.02);
        var profile = new TrapezoidProfile(m_steeringConstraints, m_steerGoal, m_steerSetpoint);
        State prevSteerSetpoint = m_steerSetpoint;
        m_steerSetpoint = profile.calculate(0.02);
        State m_nextSetpoint = profile.calculate(0.04);
        double steerVolts = m_steerPIDController.calculate(getAngle(), m_steerSetpoint.position);
        setRotationVoltage(
                steerVolts + m_steerFeedForward.calculate(m_steerSetpoint.velocity, m_nextSetpoint.velocity, 0.02)
        );
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAngle()));
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getAngle()));
    }

    public double getPinionSlip() {
        return MathUtil.angleModulus(getRelativeAngle() - getAngle());
    }
    /**
     * Reset driven distance to 0 meters.
     */
    public abstract void resetDistance();

    /**
     * Reset 
     */
    public abstract void reinitRotationEncoder();

    public void resetSteerController() {
        m_steerGoal = new State(getAngle(), 0);
        m_steerSetpoint = new State(getAngle(), 0);
    }

    

}
