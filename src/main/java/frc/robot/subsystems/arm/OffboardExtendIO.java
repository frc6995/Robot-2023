package frc.robot.subsystems.arm;

import java.util.function.Consumer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class OffboardExtendIO extends RealExtendIO {


    private SparkMaxPIDController m_sparkMaxController = m_extendMotor.getPIDController();
    public OffboardExtendIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        //TODO Auto-generated constructor stub
        m_sparkMaxController.setP(3);
        m_sparkMaxController.setI(0);
        m_sparkMaxController.setD(0.1);
        m_sparkMaxController.setFeedbackDevice(m_extendMotor.getEncoder());
    }

    @Override
    protected void setPIDFF(double length, double ffVolts) {
        // TODO Auto-generated method stub
        m_sparkMaxController.setReference(length, ControlType.kPosition, 0, ffVolts);
    }
    
}
