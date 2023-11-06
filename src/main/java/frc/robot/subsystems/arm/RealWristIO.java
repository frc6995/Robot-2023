package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.sparkmax.SparkMax;

import static frc.robot.Constants.ArmConstants.*;

public class RealWristIO extends WristIO {

    private final SparkMax m_wristMotor = new SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
    private final AbsoluteEncoder m_encoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private double velocity = 0;
    private double position = 0;
    public RealWristIO(Consumer<Runnable> addPeriodic, BooleanSupplier hasCone) {
        super(addPeriodic, hasCone);
        addPeriodic.accept(this::updateEncoder);
        m_encoder.setPositionConversionFactor(2 * Math.PI);
        m_encoder.setVelocityConversionFactor(2 * Math.PI/ 60);
        m_encoder.setInverted(true);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        m_wristMotor.setInverted(true);
        resetController();
        resetGoal();
    }

    private void updateEncoder() {
        velocity = m_encoder.getVelocity();
        position = m_encoder.getPosition();
    }
    
    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return MathUtil.angleModulus(position);
    }

    @Override
    public double getVolts() {
        return m_wristMotor.getAppliedOutput();
    }
    @Override
    protected void setVoltsInternal(double volts) {
        m_wristMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
        
    }
    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        return velocity;
    }

    // @Override
    // public boolean getHomed() {
    //     // TODO Auto-generated method stub
    //     return getAngle() < WRIST_MIN_ANGLE;
    // }

    @Override
    public void resetState(double position) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        m_wristMotor.setIdleMode(mode);
        
    }

    @Override
    public double getCurrent() {
        // TODO Auto-generated method stub
        return 0;
    }
    
}
