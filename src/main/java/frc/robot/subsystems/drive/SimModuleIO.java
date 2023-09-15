package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.util.sim.wpiClasses.QuadSwerveSim;
import frc.robot.util.sim.wpiClasses.SwerveModuleSim;
import static frc.robot.Constants.DriveConstants.*;

public class SimModuleIO extends ModuleIO {

    SwerveModuleSim moduleSim = SimModuleIO.swerveSimModuleFactory();
    private double m_steerVolts;
    private double m_driveVolts;
    public SimModuleIO(Consumer<Runnable> addPeriodic, ModuleConstants moduleConstants) {
        super(addPeriodic, moduleConstants);
        moduleSim.resetAzmth(Math.random() * 2 * Math.PI);
        // TODO Auto-generated constructor stub
    }

    static SwerveModuleSim swerveSimModuleFactory() {
        return new SwerveModuleSim(DCMotor.getNEO(1),
                DCMotor.getNEO(1),
                WHEEL_RADIUS_M,
                1.0 / AZMTH_REVS_PER_ENC_REV, // steering motor rotations per wheel steer rotation
                1.0 / WHEEL_REVS_PER_ENC_REV,
                1.0 / AZMTH_REVS_PER_ENC_REV, // same as motor rotations because NEO encoder is on motor shaft
                1.0 / WHEEL_REVS_PER_ENC_REV,
                1.5,
                2,
                ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES,
                0.01);
    }

    public SwerveModuleSim getModuleSim() {
        return moduleSim;
    }

    public void setRotationVoltage(double volts) {
        if (DriverStation.isDisabled()) {
            volts = 0;
        }
        m_steerVolts = volts;
        moduleSim.setAzmthVoltage(m_steerVolts);
    }

    public void setDriveVoltage(double volts) {
        if (DriverStation.isDisabled()) {
            volts = 0;
        }
        m_driveVolts = volts;
        moduleSim.setWheelVoltage(m_driveVolts);
    }
    @Override
    public double getAngle() {
        return MathUtil.angleModulus(moduleSim.getAzimuthEncoderPositionRev() / AZMTH_ENC_COUNTS_PER_MODULE_REV * 2 * Math.PI);
    }

    @Override
    public double getDriveDistance() {
        return moduleSim.getWheelEncoderPositionRev() / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI * WHEEL_RADIUS_M;
    }

    @Override
    public double getDriveVelocity() {
        return moduleSim.getWheelEncoderVelocityRevPerSec() / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI
                * WHEEL_RADIUS_M;
    }

    @Override
    public void resetDistance() {
        moduleSim.resetWheel(0);
    }

    @Override
    public void reinitRotationEncoder() {
    }
    @Override
    public double getDriveVoltage() {
        return m_driveVolts;
    }
    @Override
    public double getSteerVoltage() {
        return m_steerVolts;
    }

    @Override
    public double getRelativeAngle() {
        // TODO Auto-generated method stub
        return moduleSim.getAzimuthEncoderPositionRev() / AZMTH_ENC_COUNTS_PER_MODULE_REV * 2 * Math.PI;
    }

}
