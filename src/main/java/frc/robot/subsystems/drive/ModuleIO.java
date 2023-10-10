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
import autolog.Logged;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Consumer;

import autolog.AutoLog.BothLog;
import autolog.AutoLog.NTLog;

public abstract class ModuleIO implements Logged {

    // steering trapezoid profile
    protected State m_steerGoal = new State();
    protected State m_steerSetpoint = new State();
    protected double m_driveSetpoint = 0;
    protected TrapezoidProfile.Constraints m_steeringConstraints = new TrapezoidProfile.Constraints(
            STEER_MAX_SPEED_RAD_PER_SEC,
            STEER_MAX_ACCEL_RAD_PER_SEC_SQ);

    private final PIDController m_steerPIDController;
    // logging position error because it's actually the "process variable", vs its
    // derivative
    private final PIDController m_drivePIDController;
    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            Robot.isReal() ? DRIVE_FF_CONST[0] : 0,
            DRIVE_FF_CONST[1], 0.2);
            //DRIVE_FF_CONST[2]);
    private final SimpleMotorFeedforward m_steerFeedForward = new SimpleMotorFeedforward(
        0.000, STEER_KV, 0.001);
    
    private String m_loggingName;
    // steering PID controller
    // drive PID controller
    protected final ModuleConstants m_moduleConstants;
    private final Alert m_pinionSlipAlert;

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    public ModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
        m_moduleConstants = moduleConstants;
        m_loggingName = moduleConstants.name + "-[" + moduleConstants.driveMotorID + ','
                + moduleConstants.rotationMotorID + ']';
        m_pinionSlipAlert = new Alert(moduleConstants.name, "Check pinion slip", AlertType.ERROR);
        m_steerPIDController = new PIDController(
                10 / (Math.PI), 0.0, STEER_D);
        // Tell the PID controller that it can move across the -pi to pi rollover point.
        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // For a velocity controller we just use P
        // (and feedforward, which is handled in #setDesiredStateClosedLoop)
        m_drivePIDController = new PIDController(
                DRIVE_P, 0,
                DRIVE_D);
        addPeriodic.accept(this::setState);
    }

    public void updateAlerts() {
        m_pinionSlipAlert.set(Math.abs(getPinionSlip()) > Units.degreesToRadians(10));
    }

    public String getPath() {
        System.out.println(m_loggingName);
        return m_loggingName;
    }

    public abstract void setDriveVoltage(double driveVolts);

    public abstract void setRotationVoltage(double rotationVolts);

    @BothLog
    public abstract double getDriveDistance();

    @BothLog
    public abstract double getDriveVelocity();

    @BothLog
    /**
     * Returns the angle of the module from pi to -pi
     * @return
     */
    public abstract double getAngle();

    @BothLog
    public abstract double getRelativeAngle();

    @BothLog
    public abstract double getDriveVoltage();
    @BothLog
    public abstract double getSteerVoltage();

    @BothLog 
    public double getSteerSetpoint() {
        return m_steerSetpoint.position;
    }
    @BothLog
    public double getSteerGoal() {
        return m_steerGoal.position;
    }
    @BothLog
    public double getDriveSetpoint() {
        return m_driveSetpoint;
    }

    public void setDesiredState(SwerveModuleState state) {
        m_desiredState = state;
    }
    private void setState(){
        SwerveModuleState state = SwerveModuleState.optimize(m_desiredState,new Rotation2d(getAngle()));
        
        state.speedMetersPerSecond *= Math.cos(state.angle.minus(new Rotation2d(getAngle())).getRadians());
        double prevVelSetpoint = m_driveSetpoint;
        m_driveSetpoint = state.speedMetersPerSecond;
        setDrivePid(state.speedMetersPerSecond, m_driveFeedForward.calculate(prevVelSetpoint, state.speedMetersPerSecond, 0.02));

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

        setRotationPid(m_steerSetpoint.position, m_steerFeedForward.calculate(m_steerSetpoint.velocity));//m_steerFeedForward.calculate(m_steerSetpoint.velocity, m_nextSetpoint.velocity, 0.02));
    }

    public void setDrivePid(double velocity, double ffVolts){
        double pidVolts = m_drivePIDController.calculate(getDriveVelocity(), velocity);
        setDriveVoltage(pidVolts + ffVolts);
    }
    public void setRotationPid(double angle, double ffVolts){
        double pidVolts = m_steerPIDController.calculate(getAngle(), angle);
        setRotationVoltage(pidVolts + ffVolts);
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

    @NTLog
    public double getDriveCurrent() {
        return 0;
    }

    

}
