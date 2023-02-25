package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Field3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.arm.GoToPositionC;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.ArmS.ArmPosition;
import frc.robot.util.TimingTracer;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final CommandXboxController m_driverController = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final CommandOperatorKeypad m_keypad;
    private final DrivebaseS m_drivebaseS = new DrivebaseS();
    private final ArmS m_armS = new ArmS();

    private final IntakeS m_intakeS = new IntakeS();

    @Log
    private final Field2d m_field = new Field2d();
    //private final Field3d m_field3d = new Field3d();
    private final FieldObject2d m_target = m_field.getObject("target");

    private ArmPosition m_targetArmPosition = ArmConstants.STOW_POSITION;
    
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        m_keypad = new CommandOperatorKeypad(2, (pose)->m_field.getObject("Selection").setPose(pose), (position)->{m_targetArmPosition = position;});
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
        m_autoSelector.setDefaultOption("twoPiece", twoPieceAuto());
        m_field.getObject("bluePoses").setPoses(POIManager.BLUE_COMMUNITY);
        m_field.getObject("redPoses").setPoses(POIManager.RED_COMMUNITY);
        SmartDashboard.putData(m_autoSelector);
        //CameraServer.startAutomaticCapture().setResolution(320, 240);
    }

    public void configureButtonBindings() {
        //m_driverController.rightBumper().toggleOnTrue(m_drivebaseS.chasePoseC(m_target::getPose));
        //m_driverController.a().whileTrue(m_intakeS.extendAndIntakeC());
        m_driverController.y().whileTrue(
            Commands.sequence(
            m_armS.runOnce(m_armS::resetExtender),
            m_armS.run(()->{
                m_armS.setExtendLength(ArmConstants.MIN_ARM_LENGTH + Units.feetToMeters(1));
                m_armS.setPivotVelocity(0);
                m_armS.setWristVelocity(0);
            }
            ))
        );
        //m_driverController.y().whileTrue(m_armS.run(()->m_armS.setExtendVelocity(0.1)));
        m_driverController.start().whileTrue(
            m_armS.run(()->{
                m_armS.setExtendVelocity(0.1);
                m_armS.setPivotVelocity(0);
                m_armS.setWristVelocity(0);
            })
        );
        m_driverController.back().whileTrue(m_armS.run(()->{
            m_armS.setExtendVelocity(-0.1);
            m_armS.setPivotVelocity(0);
            m_armS.setWristVelocity(0);
        }
        ));

        m_driverController.a().whileTrue(
            Commands.sequence(
                m_armS.runOnce(m_armS::resetPivot),
                m_armS.run(()->{
                    m_armS.setPivotAngle(Math.PI/2);
                    m_armS.setExtendVelocity(0);
                    m_armS.setWristVelocity(0);
                }
                )));
        m_driverController.b().whileTrue(
            Commands.sequence(
                m_armS.runOnce(m_armS::resetPivot),
                m_armS.run(()->{
                    m_armS.setPivotAngle(0);
                    m_armS.setExtendVelocity(0);
                    m_armS.setWristVelocity(0);
                }
                )));
        m_driverController.leftBumper().whileTrue(
                m_armS.run(()->{
                    m_armS.setWristVolts(0.5);
                    m_armS.setExtendVelocity(0);
                    m_armS.setPivotVelocity(0);
                })
        );
        m_driverController.rightBumper().whileTrue(
            m_armS.run(()->{
                m_armS.setWristVolts(-0.5);
                m_armS.setExtendVelocity(0);
                m_armS.setPivotVelocity(0);
            })
        );
        m_keypad.stow().whileTrue(new GoToPositionC(m_armS, ()->ArmConstants.STOW_POSITION));
        m_keypad.enter().whileTrue(
            new GoToPositionC(m_armS, ()->m_targetArmPosition)
        );
    

        m_driverController.povCenter().negate().whileTrue(m_drivebaseS.run(()->{
                double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
                double adjustSpeed = 0.25; // m/s
                m_drivebaseS.driveAllianceRelative(
                    new ChassisSpeeds(
                        Math.cos(pov) * adjustSpeed,
                        Math.sin(pov) * adjustSpeed,
                        0
                    )
                );
            }
        ));
        m_operatorController.a().whileTrue(m_armS.followJointSpaceTargetC());
        m_operatorController.b().whileTrue(new GoToPositionC(m_armS, ()->ArmConstants.SCORE_HIGH_CONE_POSITION));
        m_operatorController.x().whileTrue(new GoToPositionC(m_armS, ()->ArmConstants.SCORE_MID_CONE_POSITION));
    }



    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }

    public void periodic() {
        TimingTracer.update();
        SmartDashboard.putNumber("loopTime", TimingTracer.getLoopTime());
        /* Trace the loop duration and plot to shuffleboard */

        // m_field.getObject("trajTarget").setPose(new Pose2d(
        //     m_drivebaseS.m_xController.getSetpoint(),
        //     m_drivebaseS.m_yController.getSetpoint(),
        //     Rotation2d.fromRadians(
        //     m_drivebaseS.m_thetaController.getSetpoint())
        // ));
        m_drivebaseS.drawRobotOnField(m_field);
        //m_field3d.setRobotPose(new Pose3d(m_drivebaseS.getPose()));
    }

    public void onEnabled(){
        m_drivebaseS.resetRelativeRotationEncoders();
    }

    public Command twoPieceAuto() {
        return Commands.sequence(
            m_intakeS.extendAndOuttakeC().withTimeout(1),
            Commands.deadline(
                m_drivebaseS.pathPlannerCommand(PathPlanner.loadPath("1Piece.1", 2, 2)),
                m_intakeS.extendAndIntakeC()
            ),
            m_drivebaseS.pathPlannerCommand(PathPlanner.loadPath("1Piece.2", 2, 2)),
            m_intakeS.extendAndOuttakeC().withTimeout(1)
        );
    }
}
