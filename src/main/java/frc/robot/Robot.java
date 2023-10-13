package frc.robot;

import autolog.AutoLog;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

        LiveWindow.disableAllTelemetry();
        robotContainer = new RobotContainer((fn)->this.addPeriodic(fn, kDefaultPeriod));
        //DataLogManager.logNetworkTables(true);
        addPeriodic(()->{
            AllianceWrapper.setAlliance(DriverStation.getAlliance());
        }, 0.5);
        
        System.gc();
    }

    @Override
    public void robotPeriodic() {
        if (true) {
            AutoLog.updateNT();
            AutoLog.updateDataLog();
            //NetworkTableInstance.getDefault().flush();
        }
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
    public void disabledInit() {
        robotContainer.onDisabled();
        System.gc();
    }
    @Override
    public void disabledPeriodic() {
        
    }

    public static boolean isSimulation() {
        return isSimulation;
    }

    public static boolean isReal() {
        return !isSimulation;
    }

}
