package frc.robot.subsystems.drive;

import java.util.function.Consumer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.DriveConstants.ModuleConstants;
import static frc.robot.Constants.DriveConstants.*;

public class OffboardModuleIO extends RealModuleIO {

    SparkMaxPIDController m_driveController;
    SparkMaxPIDController m_rotationController;
    public OffboardModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
        super(addPeriodic, moduleConstants);
        m_driveController = m_driveMotor.getPIDController();
        m_rotationController = m_steerMotor.getPIDController();
        m_driveController.setFeedbackDevice(m_driveMotor.getEncoder());
        m_rotationController.setFeedbackDevice(m_steerMotor.getEncoder());//m_steerMotor.getAbsoluteEncoder(Type.kDutyCycle));
        // m_rotationController.setPositionPIDWrappingEnabled(true);
        // m_rotationController.setPositionPIDWrappingMaxInput(Math.PI);
        // m_rotationController.setPositionPIDWrappingMinInput(-Math.PI);

        m_driveController.setP(0.2);
        m_driveController.setI(0);
        m_driveController.setD(0.0);
        m_driveController.setFF(DRIVE_FF_CONST[1] * (m_moduleConstants.name.charAt(0) == 'B' ? 1 : 0.5/0.51) / 12);

        m_rotationController.setP(1);
        m_rotationController.setI(0);
        m_rotationController.setD(0.01);
    }

    @Override
    public void setRotationPid(double angle, double ffVolts) {
        m_rotationController.setReference(angle,ControlType.kPosition,0,ffVolts);
    }
    @Override
    public void setDrivePid(double velocity, double ffVolts) {
        //m_driveMotor.setVoltage(ffVolts);
        m_driveController.setReference(velocity, ControlType.kVelocity, 0, ffVolts);
    }
    
}
