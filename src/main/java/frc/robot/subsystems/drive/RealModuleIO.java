package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.util.sparkmax.SparkMax;
import static frc.robot.Constants.DriveConstants.*;

public class RealModuleIO extends ModuleIO {

    private final SparkMax m_driveMotor;
    private final SparkMax m_steerMotor;
    private final AbsoluteEncoder m_magEncoder;

    public RealModuleIO(ModuleConstants moduleConstants) {
        super(moduleConstants);
        m_driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        m_steerMotor = new SparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
        m_magEncoder = m_steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // Drive motor config
        m_driveMotor.setSmartCurrentLimit(35);
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        var error = m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        System.out.println("drive config " + m_moduleConstants.driveMotorID + error.toString());
        m_driveMotor.getEncoder().setPositionConversionFactor(
                Math.PI * (WHEEL_RADIUS_M * 2) // meters/ wheel rev
                        / WHEEL_ENC_COUNTS_PER_WHEEL_REV // 1/ (enc revs / wheel rev) = wheel rev/enc rev
        );
        m_driveMotor.getEncoder().setVelocityConversionFactor(
                (WHEEL_RADIUS_M * 2) * Math.PI / 60 / WHEEL_ENC_COUNTS_PER_WHEEL_REV);

        m_driveMotor.setIdleMode(IdleMode.kBrake);

        // Steer motor config
        m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
        m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);
        m_steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        m_steerMotor.getEncoder().setPositionConversionFactor(2.0 * Math.PI * AZMTH_REVS_PER_ENC_REV);
        m_magEncoder.setPositionConversionFactor(Math.PI * 2);
        m_magEncoder.setVelocityConversionFactor(Math.PI * 2 * 60);
        m_magEncoder.setZeroOffset(m_moduleConstants.magEncoderOffset);
        m_steerMotor.setIdleMode(IdleMode.kBrake);
        // TODO Auto-generated constructor stub
    }

    @Override
    public void setDriveVoltage(double driveVolts) {
        m_driveMotor.setVoltage(driveVolts);

    }

    @Override
    public void setRotationVoltage(double rotationVolts) {
        m_steerMotor.setVoltage(rotationVolts);

    }

    @Override
    public double getDriveDistance() {
        // TODO Auto-generated method stub
        return m_driveMotor.getEncoder().getPosition();
    }

    @Override
    public double getDriveVelocity() {
        // TODO Auto-generated method stub
        return m_driveMotor.getEncoder().getVelocity();
    }

    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return MathUtil.angleModulus(m_magEncoder.getPosition());
    }

    @Override
    public double getRelativeAngle() {
        // TODO Auto-generated method stub
        return m_steerMotor.getEncoder().getPosition();
    }
    @Override
    public void resetDistance() {
        m_driveMotor.getEncoder().setPosition(0);
    }

    @Override
    public void reinitRotationEncoder() {
        m_driveMotor.getEncoder().setPosition(getAngle());
    }
    @Override
    public double getDriveVoltage() {
        return m_driveMotor.getAppliedOutput();
    }
    @Override
    public double getSteerVoltage() {
        return m_steerMotor.getAppliedOutput();
    }

}
