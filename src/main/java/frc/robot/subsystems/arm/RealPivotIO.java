package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.util.sim.SparkMaxAbsoluteEncoderWrapper;
import frc.robot.util.sparkmax.SparkMax;

import static frc.robot.Constants.ArmConstants.*;

public class RealPivotIO extends PivotIO {
    private final SparkMax m_pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax m_pivotFollowerMotor = new SparkMax(PIVOT_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxAbsoluteEncoderWrapper m_encoder;
    
    private double m_position = 0;
    public RealPivotIO(Consumer<Runnable> addPeriodic, BooleanSupplier hasCone) {
        super(addPeriodic, hasCone);
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotFollowerMotor.restoreFactoryDefaults();
        m_pivotMotor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor( 2 * Math.PI);
        m_pivotMotor.getAbsoluteEncoder(Type.kDutyCycle).setVelocityConversionFactor( 2 * Math.PI / 60.0);
        m_encoder = new SparkMaxAbsoluteEncoderWrapper(m_pivotMotor, PIVOT_ENCODER_OFFSET);
        m_pivotMotor.setInverted(true);
        m_pivotFollowerMotor.follow(m_pivotMotor, true);
        m_pivotMotor.setSmartCurrentLimit(15);
        m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
        m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
        m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 45);
        m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);

        m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        addPeriodic.accept(this::updateEncoder);
        resetController();
        resetGoal();
    }

    private void updateEncoder() {
        m_position = continuousRangeAngleModulus(m_encoder.getPosition());
    }
    @Override
    protected void setVolts(double volts) {
        m_pivotMotor.setVoltage(volts);
        
    }

    @Override
    public double getContinuousRangeAngle() {
        // TODO Auto-generated method stub
        return m_position;
    }

    public double getVolts() {
        return m_pivotMotor.getAppliedOutput();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        m_pivotMotor.setIdleMode(mode);
        m_pivotFollowerMotor.setIdleMode(mode);
        
    }

    @Override
    protected double getVelocity() {
        // TODO Auto-generated method stub
        return m_encoder.getVelocity();
    }
    
}
