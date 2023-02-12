package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.AllianceWrapper;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;

    private Command autonomousCommand;

    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
        LiveWindow.disableAllTelemetry();
        robotContainer = new RobotContainer();
        Logger.configureLoggingAndConfig(robotContainer, false);
        SmartDashboard.putNumber("vel", new SwerveModuleState().speedMetersPerSecond);
        new Trigger(DriverStation::isDSAttached).or(new Trigger(DriverStation::isFMSAttached))
        .onTrue(new InstantCommand(()->AllianceWrapper.setAlliance(DriverStation.getAlliance())));
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        robotContainer.periodic();
        Logger.updateEntries();
        
    }

    @Override
    public void autonomousInit() {
        AllianceWrapper.setAlliance(DriverStation.getAlliance());
        robotContainer.onEnabled();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) autonomousCommand.schedule();

    }

    @Override
    public void teleopInit() {
        AllianceWrapper.setAlliance(DriverStation.getAlliance());
        robotContainer.onEnabled();
        if (autonomousCommand != null) autonomousCommand.cancel();

    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().cancelAll();
    }

}
