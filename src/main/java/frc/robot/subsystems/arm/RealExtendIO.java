package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import frc.robot.Robot;
import frc.robot.subsystems.arm.ExtendIO;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import frc.robot.util.sparkmax.SparkMax;

import static frc.robot.Constants.ArmConstants.*;

import java.util.function.Consumer;

public class RealExtendIO extends ExtendIO {

    private final SparkMax m_extendMotor = new SparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxLimitSwitch m_homingSwitch = m_extendMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    private final SparkMaxEncoderWrapper m_encoder;
    private double m_position = MIN_ARM_LENGTH;
    private double m_velocity = 0;
    private boolean m_homed = false;
    /**
     * initializes extender: sets postion conversion factor of extender encoder, 
     * sets velocity conversion factor of extender encoder,
     * sets minimum and maximum soft limits for extender encoder,
     * sets initial position of encoder to minimum extension length,
     * sets profiled PID controller to minimum position
     */
    public RealExtendIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        m_extendMotor.getEncoder().setPositionConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION);
        m_extendMotor.getEncoder().setVelocityConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION / 60);
        m_extendMotor.getEncoder().setAverageDepth(4);
        m_encoder = new SparkMaxEncoderWrapper(m_extendMotor);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_LENGTH);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_LENGTH);
        m_extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_homingSwitch.enableLimitSwitch(true);
        m_extendMotor.setIdleMode(IdleMode.kCoast);
        m_extendMotor.setSmartCurrentLimit(80);
        m_encoder.setPosition(MIN_ARM_LENGTH);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        addPeriodic.accept(this::periodic);
        resetController();
        resetGoal();
    }


    @Override
    public void setVolts(double volts) {
        m_extendMotor.setVoltage(volts);
        
    }

    private void periodic() {
        double prevPosition = m_position;
        m_position = Math.max(MIN_ARM_LENGTH, m_encoder.getPosition());
        m_velocity = (m_position - prevPosition)/ 0.02;
        m_homed = m_homingSwitch.isPressed();
    }

    @Override
    public double getLength() {
        return m_position;
    }

    @Override
    public double getVelocity() {
        return m_velocity;
    }
    public boolean isHomed() {
        return m_homed;
    }
    public void onHome() {
        super.onHome();
        m_encoder.setPosition(MIN_ARM_LENGTH);
        m_extendMotor.setIdleMode(IdleMode.kBrake);
    }
    @Override
    public double getVolts() {
        return (Robot.isReal() ? 12 : 1) * m_extendMotor.getAppliedOutput();
    }


    @Override
    public void setIdleMode(IdleMode mode) {
        m_extendMotor.setIdleMode(mode);        
    }
    @Override
    public double getCurrent() {
        return m_extendMotor.getOutputCurrent();
    }
}
