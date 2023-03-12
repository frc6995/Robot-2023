package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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

    private static boolean isSimulation = false;
    
    private RobotContainer robotContainer;

    private Command autonomousCommand;

    private NetworkTableEntry matchTimeEntry = NetworkTableInstance.getDefault().getEntry("/DriverDisplay/matchTime");
    @Override
    public void robotInit() {
        Robot.isSimulation = RobotBase.isSimulation();
        DriverStation.silenceJoystickConnectionWarning(true);
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        LiveWindow.disableAllTelemetry();
        robotContainer = new RobotContainer();
        Logger.configureLoggingAndConfig(robotContainer, false);
        // new Trigger(DriverStation::isDSAttached).or(new Trigger(DriverStation::isFMSAttached))
        // .onTrue(new InstantCommand(()->AllianceWrapper.setAlliance(DriverStation.getAlliance())));
        //addPeriodic(Logger::updateEntries, 0.1);
        addPeriodic(()->{
            AllianceWrapper.setAlliance(DriverStation.getAlliance());
        }, 0.5);
    }

    @Override
    public void robotPeriodic() {
        Logger.updateEntries();
        CommandScheduler.getInstance().run();
        robotContainer.periodic();
        matchTimeEntry.setNumber(DriverStation.getMatchTime());
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

    public static boolean isSimulation() {
        return isSimulation;
    }

    public static boolean isReal() {
        return !isSimulation;
    }

}
