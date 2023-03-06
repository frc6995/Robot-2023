package frc.robot;

import java.io.Console;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.LightS;
import frc.robot.subsystems.ArmS.ArmPosition;
import frc.robot.subsystems.LightS.States;
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
    

    private final IntakeS m_intakeS = new IntakeS();
    private final ArmS m_armS = new ArmS(m_intakeS::getHandLength);

    @Log
    private final Field2d m_field = new Field2d();
    @Log
    private final Field3d m_field3d = new Field3d();
    private final FieldObject2d m_target = m_field.getObject("target");

    private ArmPosition m_targetArmPosition = ArmConstants.STOW_POSITION;
    private Pose2d m_targetAlignmentPose = new Pose2d(1.909, 1.072, Rotation2d.fromRadians(Math.PI));
    
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        m_keypad = new CommandOperatorKeypad(2, (pose)->{
            m_targetAlignmentPose = pose;
            m_field.getObject("Selection").setPose(pose);
        }, (position)->{m_targetArmPosition = position;});
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

        m_autoSelector.setDefaultOption("threePiece", highConeCubeConeBalanceAuto());


        m_field.getObject("bluePoses").setPoses(POIManager.BLUE_COMMUNITY);
        m_field.getObject("redPoses").setPoses(POIManager.RED_COMMUNITY);
        SmartDashboard.putData(m_autoSelector);
        //CameraServer.startAutomaticCapture().setResolution(320, 240);
    }

    public void configureButtonBindings() {
        m_driverController.a().toggleOnTrue(m_drivebaseS.chasePoseC(()->m_targetAlignmentPose));
        m_driverController.start().onTrue(m_intakeS.extendC());
        m_driverController.back().onTrue(m_intakeS.retractC());
        // OFFICIAL CALEB PREFERENCE
        m_driverController.rightBumper().toggleOnTrue(new GoToPositionC(m_armS, ()->ArmConstants.RAMP_CONE_INTAKE_POSITION));
        m_driverController.rightTrigger().toggleOnTrue(new GoToPositionC(m_armS, ()->ArmConstants.RAMP_CUBE_INTAKE_POSITION));

        m_driverController.leftBumper().toggleOnTrue(new GoToPositionC(m_armS, ()->ArmConstants.GROUND_CONE_INTAKE_POSITION));
        m_driverController.leftTrigger().toggleOnTrue(new GoToPositionC(m_armS, ()->ArmConstants.GROUND_CUBE_INTAKE_POSITION));
        m_driverController.y().toggleOnTrue(new GoToPositionC(m_armS, ()->ArmConstants.OVERTOP_CONE_INTAKE_POSITION).alongWith(m_intakeS.retractC()));
        m_driverController.rightStick().toggleOnTrue(new GoToPositionC(m_armS, ()->ArmConstants.PLATFORM_CONE_INTAKE_POSITION));


        

        //m_driverController.rightBumper().toggleOnTrue(m_drivebaseS.chasePoseC(()->m_targetAlignmentPose));
        //m_driverController.a().whileTrue(m_intakeS.extendAndIntakeC());
        // m_driverController.y().whileTrue(
        //     Commands.sequence(
        //     m_armS.runOnce(m_armS::resetExtender),
        //     m_armS.run(()->{
        //         m_armS.setExtendLength(ArmConstants.MIN_ARM_LENGTH + Units.feetToMeters(1));
        //         m_armS.setPivotVelocity(0);
        //         m_armS.setWristVelocity(0);
        //     }
        //     ))
        // );
        // MANUAL CONTROL
        /*
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
                    m_armS.setPivotVelocity(-0.1);
                    m_armS.setExtendVelocity(0);
                    m_armS.setWristVelocity(0);
                }
                )));
        m_driverController.y().whileTrue(
            Commands.sequence(
                m_armS.runOnce(m_armS::resetPivot),
                m_armS.run(()->{
                    m_armS.setPivotVelocity(0.1);
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
        */
        // END MANUAL CONTROL
        m_keypad.stow().onTrue(new GoToPositionC(m_armS, ()->ArmConstants.STOW_POSITION));
        m_keypad.enter().toggleOnTrue(
            new GoToPositionC(m_armS, ()->m_targetArmPosition)
            .deadlineWith(Commands.run(()->LightS.getInstance().requestState(States.Scoring)))
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
        /*
        m_driverController.a().toggleOnTrue(
            Commands.sequence(
                Commands.parallel(
                    new GoToPositionC(m_armS, ()->ArmConstants.GROUND_CUBE_INTAKE_POSITION),
                    m_intakeS.extendAndIntakeC()
                )
                .until(m_intakeS::hitCurrentLimit),
                new GoToPositionC(m_armS, ()->ArmConstants.STOW_POSITION)
            )
            
            
        );
                m_driverController.y().whileTrue(
            Commands.deadline(
                new GoToPositionC(m_armS, ()->ArmConstants.GROUND_CONE_INTAKE_POSITION),
                m_intakeS.extendC()
            ));
            // .andThen(
            //     m_intakeS.retractAndIntakeC()
            //     .withTimeout(0.75)
            // ));*/
        m_driverController.b().whileTrue(m_intakeS.intakeUntilBeamBreakC());
        m_driverController.x().whileTrue(m_intakeS.outtakeC());

        
    }



    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }

    public void periodic() {
        TimingTracer.update();
        SmartDashboard.putNumber("loopTime", TimingTracer.getLoopTime());
        /* Trace the loop duration and plot to shuffleboard */
        LightS.getInstance().periodic();
        m_drivebaseS.drawRobotOnField(m_field);
        m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
        m_field3d.setRobotPose(new Pose3d(m_drivebaseS.getPose().getX(), m_drivebaseS.getPose().getY(), 0, m_drivebaseS.getRotation3d()));
    }

    public void onEnabled(){
        m_drivebaseS.resetRelativeRotationEncoders();
    }

    //Autonomous Commands:

    public Command highConeCubeConeBalanceAuto() {
        var pathGroup = PathPlanner.loadPathGroup("3PieceGroup", new PathConstraints(3, 2.5),  new PathConstraints[0]);
        return Commands.sequence(
            m_intakeS.retractC(),
            Commands.sequence(
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                m_intakeS.outtakeC().withTimeout(1),
                m_armS.goToPositionC(ArmConstants.STOW_POSITION)
            ),
            Commands.sequence(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds()))),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION),
                m_intakeS.intakeUntilBeamBreakC().withTimeout(1),
                m_armS.goToPositionC(ArmConstants.STOW_POSITION)
            ),
            Commands.sequence(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(1)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds()))),
                m_armS.goToPositionC(ArmConstants.HYBRID_NODE_OUTTAKE_POSITION),
                m_intakeS.intakeUntilBeamBreakC().withTimeout(1)
            ),

            m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))

        );
    }

    

    public Command EighteenPointAuto(){
        
        return Commands.sequence(
            
        );
    }

    //Bump:

    public Command BumpFifteenPointAuto(){
        
        return Commands.sequence(
            
        );
    }

    public Command BumpTwentyonePointAutoNo2nd(){
        
        return Commands.sequence(
            
        );
    }

    public Command BumpTwentyonePointAutoWith2nd(){

        return Commands.sequence(
            
        );
    }

    public Command BumpTwentysevenPointAuto(){

        return Commands.sequence(
            
        );
    }

    //No Bump

    public Command FifteenPointAuto(){
        
        return Commands.sequence(
            
        );
    }

    public Command TwentyonePointAutoNo2nd(){
        
        return Commands.sequence(
            
        );
    }

    public Command TwentyonePointAutoWith2nd(){

        return Commands.sequence(
            
        );
    }

    public Command TwentysevenPointAuto(){

        return Commands.sequence(
            
        );
    }
}
