package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.drive.SecondOrderSwerveModuleState;
import frc.robot.util.sim.DutyCycleEncoderSim;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveModule extends SubsystemBase implements Loggable{

    /**
     * Class to represent and handle a swerve module
     * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
     * for both rotation and linear movement
     */

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_steerMotor;

    // These wrappers handle sim value holding, as well as circumventing the Spark MAX velocity signal delay
    // by integrating position. Credit for the latter to 6328. 
    private final SparkMaxEncoderWrapper m_driveEncoderWrapper;
    private final SparkMaxEncoderWrapper m_steerEncoderWrapper;


    private final DutyCycleEncoder m_magEncoder;
    private final DutyCycleEncoderSim m_magEncoderSim;
    private final double m_magEncoderOffset;

    //absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on

    private final ProfiledPIDController m_steerPIDController;
    // logging position error because it's actually the "process variable", vs its derivative
    private final PIDController m_drivePIDController;
    private final String m_loggingName;
    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
        DRIVE_FF_CONST[0],
        DRIVE_FF_CONST[1],
        DRIVE_FF_CONST[2]);        


    public SwerveModule( ModuleConstants moduleConstants) {
        m_driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults(false);
        m_steerMotor.restoreFactoryDefaults(false);

        //set the output of the drive encoder to be in meters (instead of motor rots) for linear measurement
        // wheel diam * pi = wheel circumference (meters/wheel rot) *
        // 1/6.86 wheel rots per motor rot *
        // number of motor rots
        // = number of meters traveled
        m_driveMotor.getEncoder().setPositionConversionFactor(
            Math.PI * (WHEEL_RADIUS_M * 2) // meters/ wheel rev
            / WHEEL_ENC_COUNTS_PER_WHEEL_REV // 1/ (enc revs / wheel rev) = wheel rev/enc rev
        );

        //set the output of the drive encoder to be in meters per second (instead of motor rpm) for velocity measurement
        // wheel diam * pi = wheel circumference (meters/wheel rot) *
        // 1/60 minutes per sec *
        // 1/5.14 wheel rots per motor rot *
        // motor rpm = wheel speed, m/s
        m_driveMotor.getEncoder().setVelocityConversionFactor(
            (WHEEL_RADIUS_M * 2) * Math.PI / 60 / WHEEL_ENC_COUNTS_PER_WHEEL_REV
        );

        //set the output of the rotation encoder to be in radians
        // (2pi rad/(module rotation)) / 12.8 (motor rots/module rots)
        m_steerMotor.getEncoder().setPositionConversionFactor(2.0 * Math.PI * AZMTH_REVS_PER_ENC_REV);

        // Create the encoder wrappers after setting conversion factors so that the wrapper reads the conversions.
        m_driveEncoderWrapper = new SparkMaxEncoderWrapper(m_driveMotor);
        m_steerEncoderWrapper = new SparkMaxEncoderWrapper(m_steerMotor);
        //Config the mag encoder, which is directly on the module rotation shaft.
        m_magEncoder = new DutyCycleEncoder(moduleConstants.magEncoderID);
        //magEncoder.setDistancePerRotation(2*Math.PI);
        m_magEncoder.setDutyCycleRange(1.0/4098.0, 4096.0/4098.0); //min and max pulse width from the mag encoder datasheet
        m_magEncoderOffset = moduleConstants.magEncoderOffset;
        //magEncoder.setPositionOffset(measuredOffsetRadians/(2*Math.PI));
        // The magnet in the module is not aligned straight down the direction the wheel points, but it is fixed in place.
        // This means we can subtract a fixed position offset from the encoder reading,
        // I.E. if the module is at 0 but the magnet points at 30 degrees, we can subtract 30 degrees from all readings
        //magEncoder.setPositionOffset(measuredOffsetRadians/(2*Math.PI));
        
        //Allows us to set what the mag encoder reads in sim.
        // Start with what it would read if the module is forward.
        m_magEncoderSim = new DutyCycleEncoderSim(m_magEncoder);
        m_magEncoderSim.setAbsolutePosition(m_magEncoderOffset/ (2*Math.PI));

        //Drive motors should brake, rotation motors should coast (to allow module realignment)
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_steerMotor.setIdleMode(IdleMode.kCoast);

        // Config the pid controllers

        // For a position controller we use a P loop on the position error
        // and a D loop, which is P on the derivative/rate of change of the position error
        // Theoretically, if the error is increasing (aka, the setpoint is getting away),
        // we should match the velocity of the setpoint with our D term to stabilize the error,
        // then add the additional output proportional to the size of the error.
        // Trapezoid Profile Constraints: 7.8 rot/s (limit of the NEO), 40 rot/s^2
        m_steerPIDController = new ProfiledPIDController(
            STEER_P, 0.0, STEER_D,
            new TrapezoidProfile.Constraints(
                STEER_MAX_SPEED_RAD_PER_SEC,
                STEER_MAX_ACCEL_RAD_PER_SEC_SQ));
        // Tell the PID controller that it can move across the -pi to pi rollover point.
        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // For a velocity controller we just use P
        // (and feedforward, which is handled in #setDesiredStateClosedLoop)
        m_drivePIDController = new PIDController(
            DRIVE_P, 0, 
            DRIVE_D);
        // Give this module a unique name on the dashboard so we have four separate sub-tabs.
        m_loggingName = "SwerveModule-" + moduleConstants.name + "-[" + m_driveMotor.getDeviceId() + ',' + m_steerMotor.getDeviceId() + ']';
        resetDistance();
    }

    /**
     * We override this method to setup Oblog with the module's unique name.
     */
    public String configureLogName() {
        System.out.println(m_loggingName);
        return m_loggingName;
    }
    /**
     * Reset the driven distance to 0.
     */
    public void resetDistance() {
        m_driveEncoderWrapper.setPosition(0);
    }

    /**
     * Returns the distance driven by the module in meters since the last reset.
     * @return the distance in meters.
     */
    public double getDriveDistanceMeters() {
        return m_driveEncoderWrapper.getPosition();
    }
    
    /**
     * Returns the current angle of the module in radians, from the mag encoder.
     * @return a Rotation2d, where 0 is forward and pi/-pi is backward.
     */
    @Log(methodName = "getRadians")
    public Rotation2d getMagEncoderAngle() {
        double unsignedAngle = m_magEncoder.getAbsolutePosition() * 2*Math.PI - m_magEncoderOffset;
        return new Rotation2d(unsignedAngle);
    }

    /**
     * Returns the current angle of the module in radians, from the rotation NEO built-in encoder.
     * The sim model is immediate and perfect response, which is to say that in sim,
     * current angle is always desired angle.
     * @return a Rotation2d, where 0 is forward and pi/-pi is backward.
     */
    @Log(methodName = "getRadians")
    public Rotation2d getCanEncoderAngle() {
        return new Rotation2d(m_steerEncoderWrapper.getPosition());
    }

    /**
     * Returns the current velocity of the module in meters per second.
     * The sim model is immediate and perfect response, which is to say that in sim,
     * current velocity is always desired velocity.
     * @return
     */
    @Log
    public double getCurrentVelocityMetersPerSecond() {
        return m_driveEncoderWrapper.getVelocity();
    }

    public double getAppliedDriveVoltage() {
        return m_driveMotor.getAppliedOutput();
    }

    public double getAppliedRotationVoltage() {
        return m_steerMotor.getAppliedOutput();
    }


    /**
     * Initialize the integrated mag encoder
     * The mag encoder will read a (magnet offset) + (module offset from forward)
     * But we subtract the magnet offset in the encoder library, so when starting up, the encoder will report
     * the module offset from forward.
    */
    public void initRotationOffset() {
        m_steerEncoderWrapper.setPosition(getMagEncoderAngle().getRadians());
    }

    /**
     * Method to set the desired state of the swerve module
     * Parameter: 
     * SwerveModuleState object that holds a desired linear and rotational setpoint
     * Uses PID and a feedforward to control the output
     */
    public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {

        // Save the desired state for reference (Simulation assumes the modules always are at the desired state)
        setDesiredStateClosedLoop(
            new SecondOrderSwerveModuleState(
                desiredState.speedMetersPerSecond, desiredState.angle,
                0, 0));
    }

    public void setDesiredStateClosedLoop(SecondOrderSwerveModuleState desiredState) {

        // Save the desired state for reference (Simulation assumes the modules always are at the desired state)
        
        desiredState = SecondOrderSwerveModuleState.optimize(desiredState, getCanEncoderAngle());
        
        double goal = desiredState.angle.getRadians();
        double measurement = getCanEncoderAngle().getRadians();
        double rotationVolts = m_steerPIDController.calculate(measurement, goal);
        if (RobotBase.isReal()) {
            rotationVolts += 0.1;
            rotationVolts += 0.5 * desiredState.omegaRadiansPerSecond;
        }
        double driveVolts = m_drivePIDController.calculate(getCurrentVelocityMetersPerSecond(), desiredState.speedMetersPerSecond)
            + m_driveFeedForward.calculate(desiredState.speedMetersPerSecond, desiredState.accelerationMetersPerSecondSquared);
            //(this.desiredState.speedMetersPerSecond - previousState.speedMetersPerSecond) / 0.02);
        m_steerMotor.setVoltage(rotationVolts);
        m_driveMotor.setVoltage(driveVolts);
    }

    public void periodic() {
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                getCurrentVelocityMetersPerSecond(),
                getCanEncoderAngle());
    }
    

    /**
     * Resets drive and rotation encoders to 0 position. (in sim and irl)
     */
    public void resetEncoders() {
        initRotationOffset();
        m_driveEncoderWrapper.setPosition(0);
    }
    
    /**
     * Set the state of the module as specified by the simulator
     * @param angle_rad
     * @param wheelPos_m
     * @param wheelVel_mps
     */
    public void setSimState(double angle_rad, double wheelPos_m, double wheelVel_mps) {
        m_steerEncoderWrapper.setSimPosition(angle_rad);
        m_driveEncoderWrapper.setSimPosition(wheelPos_m);
        m_driveEncoderWrapper.setSimVelocity(wheelVel_mps);

        m_magEncoderSim.setAbsolutePosition((angle_rad +m_magEncoderOffset)/ (2*Math.PI));
    }

    @Log
    public double getSteerSetpoint() {
        return m_steerPIDController.getSetpoint().position;
    }

    @Log
    public double getVelocitySetpoint() {
        return m_drivePIDController.getSetpoint();
    }
}