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

import frc.robot.subsystems.arm.ExtendIO;
import frc.robot.util.sparkmax.SparkMax;

import static frc.robot.Constants.ArmConstants.*;

import java.util.function.Consumer;

public class RealExtendIO extends ExtendIO {

    private final SparkMax m_extendMotor = new SparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxLimitSwitch m_homingSwitch = m_extendMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    private final RelativeEncoder m_encoder = m_extendMotor.getEncoder();
    private double m_position = MIN_ARM_LENGTH;
    private double m_velocity = 0;
    /**
     * initializes extender: sets postion conversion factor of extender encoder, 
     * sets velocity conversion factor of extender encoder,
     * sets minimum and maximum soft limits for extender encoder,
     * sets initial position of encoder to minimum extension length,
     * sets profiled PID controller to minimum position
     */
    public RealExtendIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        m_encoder.setPositionConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION);
        m_encoder.setVelocityConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION / 60);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_LENGTH);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_LENGTH);
        m_extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_homingSwitch.enableLimitSwitch(true);
        m_extendMotor.setIdleMode(IdleMode.kCoast);
        m_extendMotor.setSmartCurrentLimit(40);
        m_encoder.setPosition(MIN_ARM_LENGTH);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
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
        m_position = Math.max(MIN_ARM_LENGTH-0.25, m_encoder.getPosition());
        m_velocity = m_encoder.getVelocity();
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
        return m_homingSwitch.isPressed();
    }
    public void onHome() {
        super.onHome();
        m_encoder.setPosition(MIN_ARM_LENGTH);
        m_extendMotor.setIdleMode(IdleMode.kBrake);
    }
    @Override
    public double getVolts() {
        return m_extendMotor.getAppliedOutput();
    }
}
