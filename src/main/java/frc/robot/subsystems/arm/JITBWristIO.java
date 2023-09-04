package frc.robot.subsystems.arm;

import java.util.function.Consumer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.sparkmax.SparkMax;

import static frc.robot.Constants.ArmConstants.*;

public class JITBWristIO extends WristIO {

    private final SparkMax m_wristMotor = new SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_wristMotor.getEncoder();
    private final Constraints m_homingConstraints = new Constraints(0.5, 0.5);
    //private final AbsoluteEncoder m_encoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private final double WRIST_RANGE = WRIST_MAX_ANGLE - WRIST_MIN_ANGLE;
    private boolean isHoming = true;
    public JITBWristIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        m_encoder.setPositionConversionFactor(2 * Math.PI * WRIST_ROTATIONS_PER_MOTOR_ROTATION);
        m_encoder.setVelocityConversionFactor(2 * Math.PI * WRIST_ROTATIONS_PER_MOTOR_ROTATION/ (60));
        m_encoder.setInverted(true);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) WRIST_MIN_ANGLE);
        m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) WRIST_MAX_ANGLE);
        m_wristMotor.setInverted(true);
        startHome();
        resetController();
        resetGoal();
        new Trigger(()->isHoming && getHomed()).onTrue(Commands.runOnce(this::endHome));
    }
    
    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return MathUtil.angleModulus(m_encoder.getPosition());
    }

    @Override
    public void resetState(double position) {
        m_encoder.setPosition(position);
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
        return m_encoder.getVelocity();
    }

    @Override
    public boolean getHomed() {
        return m_wristMotor.getOutputCurrent() > 10;
    }


    @Override
    public Constraints getConstraints() {
        return isHoming ? m_homingConstraints : m_constraints;
    }

    
}
