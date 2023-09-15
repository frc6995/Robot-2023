package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.NavX.AHRS;

public class RealSwerveDriveIO extends SwerveDriveIO {
    public RealSwerveDriveIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        m_modules = new ArrayList<>();
        Timer.delay(0.05);
        m_modules.add(new OffboardModuleIO(addPeriodic, ModuleConstants.FL));
        Timer.delay(0.05);
        m_modules.add(new OffboardModuleIO(addPeriodic, ModuleConstants.FR));
        Timer.delay(0.05);
        m_modules.add(new OffboardModuleIO(addPeriodic, ModuleConstants.BL));
        Timer.delay(0.05);
        m_modules.add(new OffboardModuleIO(addPeriodic, ModuleConstants.BR));
        // TODO Auto-generated constructor stub
    }

    public void resetPose(Pose2d pose) {
    };

    @Override
    public void resetIMU() {
        m_navx.reset();
    }

    @Override
    public Pose2d getSimPose() {
        return new Pose2d();
    }

}
