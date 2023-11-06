package frc.robot.subsystems.drive;

import java.util.function.Consumer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.DriveConstants.ModuleConstants;

public class OffboardModuleIO extends RealModuleIO {

    SparkMaxPIDController m_driveController;
    SparkMaxPIDController m_rotationController;
    public OffboardModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
        super(addPeriodic, moduleConstants);
        m_driveController = m_driveMotor.getPIDController();
        m_rotationController = m_steerMotor.getPIDController();
        m_driveController.setFeedbackDevice(m_driveMotor.getEncoder());
        m_rotationController.setFeedbackDevice(m_steerMotor.getAbsoluteEncoder(Type.kDutyCycle));
        m_rotationController.setPositionPIDWrappingEnabled(true);
        m_rotationController.setPositionPIDWrappingMaxInput(Math.PI);
        m_rotationController.setPositionPIDWrappingMinInput(-Math.PI);

        m_driveController.setP(0.1);
        m_driveController.setI(0);
        m_driveController.setD(0.005);

        m_rotationController.setP(0.5);
        m_rotationController.setI(0);
        m_rotationController.setD(0);
    }

    @Override
    public void setRotationPid(double angle, double ffVolts) {
        m_rotationController.setReference(angle,ControlType.kPosition,0,ffVolts);
    }
    @Override
    public void setDrivePid(double velocity, double ffVolts) {
        m_driveController.setReference(velocity, ControlType.kVelocity, 0, ffVolts);
    }
    
}
