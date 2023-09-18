package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.NavX.AHRS;
import io.github.oblarg.oblog.annotations.Log;
import autolog.Logged;
import autolog.AutoLog.BothLog;

import static frc.robot.Constants.DriveConstants.*;

public abstract class SwerveDriveIO implements Logged {
    protected final AHRS m_navx = new AHRS(Port.kMXP, (byte) 50);
    protected List<ModuleIO> m_modules;

    private SwerveModuleState[] currentStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
    private SwerveModulePosition[] currentPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    public SwerveDriveIO(Consumer<Runnable> addPeriodic) {
        addPeriodic.accept(this::periodic);
        m_navx.reset();
        m_navx.enableLogging(true);
    }


    @Override
    public String getPath() {
        return "io";
    }
    @BothLog
    public Rotation2d getGyroHeading() {
        return m_navx.getRotation2d();
    }

    private void periodic() {
        updateModulePositions();
        updateModuleStates();
        //m_modules.forEach(ModuleIO::updateAlerts);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < NUM_MODULES; i++) {
            m_modules.get(i).setDesiredState(moduleStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        return currentStates;
    }

    public SwerveModulePosition[] getCurrentPositions() {
        return currentPositions;
    }

    private void updateModuleStates() {
        for (int i = 0; i < NUM_MODULES; i++) {
            currentStates[i] = m_modules.get(i).getCurrentState();
        }
    }

    private void updateModulePositions() {
        for (int i = 0; i < NUM_MODULES; i++) {
            currentPositions[i] = m_modules.get(i).getCurrentPosition();
        }
    }

    public void resetDistances() {
        m_modules.forEach((module) -> module.resetDistance());
    }

    public abstract void resetPose(Pose2d pose);

    public abstract void resetIMU();

    public void reinitRotationEncoders() {
        m_modules.forEach(ModuleIO::reinitRotationEncoder);
    }
    public void resetModuleSteerControllers() {
        m_modules.forEach(ModuleIO::resetSteerController);
    }

    public abstract Pose2d getSimPose();

    public Rotation3d getRotation3d() {
        return new Rotation3d(new Quaternion(
                m_navx.getQuaternionW(),
                m_navx.getQuaternionX(),
                m_navx.getQuaternionY(),
                m_navx.getQuaternionZ()));
    }

    public double getPitch() {
        return Units.degreesToRadians(m_navx.getPitch());
    }
}
