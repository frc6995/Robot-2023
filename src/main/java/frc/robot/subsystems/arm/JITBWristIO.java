package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import frc.robot.util.sparkmax.SparkMax;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

public class JITBWristIO extends WristIO {

    private final SparkMax m_wristMotor = new SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_encoder;
    private final SparkMaxPIDController m_pidController = m_wristMotor.getPIDController();
    private final Constraints m_homingConstraints = new Constraints(1, 1);
    //private final AbsoluteEncoder m_encoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private final double WRIST_RANGE = WRIST_MAX_ANGLE - WRIST_MIN_ANGLE;
    private double m_current;
    private double m_velocity;
    private double m_volts;
    private double m_angle;
    private Debouncer m_homingDebouncer = new Debouncer(0.5);
    public JITBWristIO(Consumer<Runnable> addPeriodic, BooleanSupplier hasCone) {
        super(addPeriodic, hasCone);
        m_wristMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * WRIST_ROTATIONS_PER_MOTOR_ROTATION);
        m_wristMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI * WRIST_ROTATIONS_PER_MOTOR_ROTATION/ (60));
        m_encoder = new SparkMaxEncoderWrapper(m_wristMotor);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        m_pidController.setFeedbackDevice(m_wristMotor.getEncoder());
        m_pidController.setPositionPIDWrappingEnabled(false);

        m_pidController.setP(1);
        m_pidController.setI(0);
        m_pidController.setD(0);
        m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) WRIST_MIN_ANGLE);
        m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) WRIST_MAX_ANGLE);
        m_wristMotor.setSmartCurrentLimit(40);
        m_wristMotor.setInverted(true);
        startHome();
        resetController();
        resetGoal();
        new Trigger(()->isHoming && getHomed()).onTrue(Commands.runOnce(this::endHome));
        addPeriodic.accept(this::updateEncoder);
    }
    
    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return m_angle;
    }

    @Override
    public void resetState(double position) {
        m_encoder.setPosition(position);
    }

    private void updateEncoder() {
        m_angle = m_encoder.getPosition();
        m_volts = m_wristMotor.getAppliedOutput() * 12;
        m_velocity = m_encoder.getVelocity();
        m_current = m_wristMotor.getOutputCurrent();
    }

    public void setPIDFF(double position, double ffVolts) {
        if (isHoming) {
            m_wristMotor.setVoltage(3);
        } else {
            m_pidController.setReference(position, ControlType.kPosition, 0, ffVolts);
        }
        
    }
    @Override
    public double getVolts() {
        return m_volts;
    }
    @Override
    protected void setVoltsInternal(double volts) {
        m_wristMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
        
    }

    @Override
    public boolean getHomed() {
        return super.getHomed() || m_homingDebouncer.calculate(m_current > 15);
    }
    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        return m_velocity;
    }

    public void resetController() {
        super.resetController();
    }

    @Override
    public Constraints getConstraints() {
        return isHoming ? m_homingConstraints : m_constraints;
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        m_wristMotor.setIdleMode(mode);
        
    }

    @Override
    public double getCurrent() {
        // TODO Auto-generated method stub
        return m_current;
    }

    
}
