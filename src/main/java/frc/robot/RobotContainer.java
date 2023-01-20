package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Field3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final CommandXboxController m_driverController = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    private final DrivebaseS m_drivebaseS = new DrivebaseS();

    @Log
    private final Field2d m_field = new Field2d();
    @Log
    private final Field3d m_field3d = new Field3d();
    private final FieldObject2d m_target = m_field.getObject("target");
    
    @Log
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

    @Log
    private double m_lastLoopDuration;
    private double m_lastLoopTimestampSeconds;

    public RobotContainer() {
        m_target.setPose(new Pose2d(1.909, 1.072, Rotation2d.fromRadians(Math.PI)));
        
        
        m_drivebaseS.setDefaultCommand(
            new OperatorControlC(
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                m_driverController::getRightX,
                m_drivebaseS
            )
        );

        configureButtonBindings();
        PathPlannerTrajectory sCurveTrajectory = PathPlanner.loadPath("StraightBack", 2.5, 2.5, false);
        m_field.getObject("traj").setTrajectory((Trajectory) sCurveTrajectory);
        m_autoSelector.setDefaultOption("sCurve",
            m_drivebaseS.pathPlannerCommand(
                sCurveTrajectory
            ).beforeStarting(
                m_drivebaseS.runOnce(
                    ()->m_drivebaseS.resetPose(sCurveTrajectory.getInitialHolonomicPose())
                )
            )
        );
        SmartDashboard.putData(m_autoSelector);
    }

    public void configureButtonBindings() {
        new Trigger(RobotController::getUserButton).onTrue(runOnce(()->m_drivebaseS.resetPose(new Pose2d())));
        m_driverController.povCenter().onFalse(
            runOnce(
                ()->m_drivebaseS.setRotationState(
                    Units.degreesToRadians(m_driverController.getHID().getPOV()))
            )
        );
        m_driverController.a().toggleOnTrue(m_drivebaseS.chasePoseC(m_target::getPose));
    }

    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }

    public void periodic() {
        /* Trace the loop duration and plot to shuffleboard */
        m_lastLoopDuration = (WPIUtilJNI.now() / 1e6) - m_lastLoopTimestampSeconds;
        m_lastLoopTimestampSeconds += m_lastLoopDuration;
        m_field.getObject("trajTarget").setPose(new Pose2d(
            m_drivebaseS.m_xController.getSetpoint(),
            m_drivebaseS.m_yController.getSetpoint(),
            Rotation2d.fromRadians(
            m_drivebaseS.m_thetaController.getSetpoint())
        ));
        m_drivebaseS.drawRobotOnField(m_field);
        m_field3d.setRobotPose(new Pose3d(m_drivebaseS.getPose()));
    }

    public void onEnabled(){
        m_drivebaseS.resetRelativeRotationEncoders();
    }
}
