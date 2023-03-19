package frc.robot;
import java.io.Console;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Field3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.POIManager.POIS;
import frc.robot.commands.arm.GoToPositionC;
import frc.robot.commands.arm.HoldCurrentPositionC;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.commands.drivetrain.OperatorControlHeadingC;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightS;
import frc.robot.subsystems.ArmS.ArmPosition;
import frc.robot.subsystems.LightS.States;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;
import io.github.oblarg.oblog.annotations.Log;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */
    private final CommandXboxController m_driverController = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    //private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final CommandOperatorKeypad m_keypad;
    private final DrivebaseS m_drivebaseS = new DrivebaseS();
    

    private final IntakeS m_intakeS = new IntakeS();
    private final ArmS m_armS = new ArmS(m_intakeS::getHandLength);

    @Log
    private final Field2d m_field = new Field2d();
    // @Log
    // private final Field3d m_field3d = new Field3d();
    private final FieldObject2d m_target = m_field.getObject("target");

    private ArmPosition m_targetArmPosition = ArmConstants.STOW_POSITION;
    private Pose2d m_targetAlignmentPose = new Pose2d(1.909, 1.072, Rotation2d.fromRadians(Math.PI));
    private boolean m_isCubeSelected = true;
    
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        m_keypad = new CommandOperatorKeypad(2, (pose)->{
            m_targetAlignmentPose = pose;
            m_field.getObject("Selection").setPose(pose);
        }, (position)->{m_targetArmPosition = position;}, (selectedCube)->{m_isCubeSelected = selectedCube;});
        m_keypad.setpointCommand(0, 0).schedule();
        m_target.setPose(new Pose2d(1.909, 1.072, Rotation2d.fromRadians(Math.PI)));
        
        
        m_drivebaseS.setDefaultCommand(
            new OperatorControlC(
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                m_driverController::getRightX,
                ()->{
                    double downfield = (AllianceWrapper.getAlliance() == Alliance.Red) ? 
                    Math.PI : 0.0;
                    if (m_driverController.start().getAsBoolean()) {
                        return downfield;
                    }
                    else {
                        return m_drivebaseS.getPoseHeading().getRadians();
                    }
                },
                m_driverController.start(),
                m_drivebaseS
            )
        );

        configureButtonBindings();

        // m_driverController.start().whileTrue(m_drivebaseS.chargeStationBatteryFirstC());
        // m_driverController.back().whileTrue(m_drivebaseS.chargeStationFrontFirstC());

        //Autonomous Option Selections:
        m_autoSelector.setDefaultOption("18 Point", eighteenPointAuto());
        //No Bump:
        m_autoSelector.addOption("15 Point", fifteenPointAuto());
        m_autoSelector.addOption("21 Point No 2nd", twentyonePointAutoNo2nd());
        m_autoSelector.addOption("21 Point With 2nd", twentyonePointAutoWith2nd());
        m_autoSelector.addOption("27 Point", twentysevenPointAuto());


        m_field.getObject("bluePoses").setPoses(POIManager.BLUE_COMMUNITY);
        m_field.getObject("redPoses").setPoses(POIManager.RED_COMMUNITY);
        SmartDashboard.putData(m_autoSelector);
        //CameraServer.startAutomaticCapture().setResolution(320, 240);
    }

    public void configureButtonBindings() {
        m_driverController.a().toggleOnTrue(m_drivebaseS.chasePoseC(()->m_targetAlignmentPose));
        m_driverController.b().toggleOnTrue(
            Commands.sequence(
                m_intakeS.outtakeC().withTimeout(0.5),
                m_armS.stowIndefiniteC()
            )
            );
        //m_driverController.back().onTrue(m_intakeS.retractC());
        // OFFICIAL CALEB PREFERENCE
        m_driverController.y().toggleOnTrue(armIntakeCG(ArmConstants.OVERTOP_CONE_INTAKE_POSITION, false));
        m_driverController.x().onTrue(m_armS.stowIndefiniteC());



        m_driverController.rightBumper().toggleOnTrue(armIntakeSelectedCG(
            ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT, 
            ArmConstants.RAMP_CONE_INTAKE_POSITION, ()->m_isCubeSelected));

        m_driverController.rightTrigger().toggleOnTrue(armIntakeSelectedCG(
            ArmConstants.GROUND_CUBE_INTAKE_POSITION, 
            ArmConstants.GROUND_CONE_INTAKE_POSITION, ()->m_isCubeSelected));
            
        m_driverController.leftBumper().toggleOnTrue(armIntakeSelectedCG(
            ArmConstants.PLATFORM_CUBE_INTAKE_POSITION, 
            ArmConstants.PLATFORM_CONE_INTAKE_POSITION, ()->m_isCubeSelected));
        
        // m_driverController.leftTrigger().whileTrue(new ConditionalCommand(
        //     m_drivebaseS.chasePoseC(()->POIManager.ownPOI(POIS.CUBE_RAMP)),
        //     m_drivebaseS.chasePoseC(()->POIManager.ownPOI(POIS.CONE_RAMP)),
        //     ()->m_isCubeSelected));

        m_keypad.stow().onTrue(m_intakeS.intakeC().withTimeout(0.25));
        m_keypad.enter().toggleOnTrue(
            sequence(
                new GoToPositionC(m_armS, ()->m_targetArmPosition),
                either(
                    sequence(
                        m_intakeS.outtakeC().withTimeout(0.4),
                        m_armS.stowC()
                    ),
                none(), 
                ()->m_isCubeSelected)
            )
            .deadlineWith(Commands.run(()->LightS.getInstance().requestState(States.Scoring)))
        );
    

        m_driverController.povCenter().negate().whileTrue(m_drivebaseS.run(()->{
                double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
                double adjustSpeed = 0.5; // m/s
                m_drivebaseS.driveAllianceRelative(
                    new ChassisSpeeds(
                        Math.cos(pov) * adjustSpeed,
                        Math.sin(pov) * adjustSpeed,
                        0
                    )
                );
            }
        ));        
    }



    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }

    public void periodic() {
        TimingTracer.update();
        //SmartDashboard.putNumber("loopTime", TimingTracer.getLoopTime());
        LightS.getInstance().requestState(m_isCubeSelected? States.RequestingCube : States.RequestingCone);
        /* Trace the loop duration and plot to shuffleboard */
        LightS.getInstance().periodic();
        m_drivebaseS.drawRobotOnField(m_field);
        //m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
        ///]m_field3d.setRobotPose(new Pose3d(m_drivebaseS.getPose().getX(), m_drivebaseS.getPose().getY(), 0, m_drivebaseS.getRotation3d()));
    }

    public void onEnabled(){
        m_drivebaseS.resetRelativeRotationEncoders();
    }

    public Command armIntakeCG(ArmPosition position, boolean isCube) {
        return 
        Commands.sequence(
            Commands.deadline(
                m_intakeS.setGamePieceC(()->isCube).andThen(m_intakeS.intakeUntilBeamBreakC()),
                m_armS.goToPositionIndefiniteC(position)

            ),
            Commands.parallel(m_armS.stowIndefiniteC(), 
            Commands.run(()->LightS.getInstance().requestState(isCube ? States.IntakedCube : States.IntakedCone)).asProxy().withTimeout(0.75)
            )
        );
    }

    

    public Command armIntakeSelectedCG(ArmPosition cubePosition, ArmPosition conePosition, BooleanSupplier isCube) {
        return Commands.either(
            armIntakeCG(cubePosition, true), armIntakeCG(conePosition, false), isCube);
    }


    //Autonomous Commands:
    

    public Command eighteenPointAuto(){
        
        return Commands.sequence(
            m_intakeS.retractC(),

            Commands.runOnce(
                ()->m_drivebaseS.resetPose(
                    NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(3), AllianceWrapper.getAlliance())
                )),
            m_drivebaseS.stopC(),
            m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3).asProxy(),
            m_intakeS.outtakeC().withTimeout(0.4),

            m_armS.goToPositionC(ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT).asProxy(),
            m_drivebaseS.chasePoseC(()->new Pose2d(
                POIS.CHARGE_STATION.ownPose().getX(),
                m_drivebaseS.getPose().getY(),
                POIS.CHARGE_STATION.ownPose().getRotation())
            )
            //m_drivebaseS.chargeStationBatteryFirstC()
        ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()));
    }





    //No Bump

    public Command fifteenPointAuto(){
        
        var pathGroup = PathPlanner.loadPathGroup("15 Point", new PathConstraints(2, 2), new PathConstraints(1, 1), new PathConstraints(2, 2));
        return Commands.sequence(
            m_intakeS.retractC(),

            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3),
            m_intakeS.outtakeC().withTimeout(0.4),

            Commands.parallel(
                m_armS.goToPositionC(ArmConstants.OVERTOP_CONE_INTAKE_POSITION).asProxy().withTimeout(6),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
                //m_drivebaseS.pathPlannerCommand(singlePath)
            ),
            
            Commands.deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(1)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds()))), 
                m_intakeS.intakeUntilBeamBreakC()),

            Commands.parallel(
                m_intakeS.intakeC().withTimeout(0.1),
                m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
            ),

            Commands.parallel(
                m_intakeS.intakeC().withTimeout(0.1),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3)
            ),

            m_intakeS.outtakeC().withTimeout(0.4)
        );
    }

    public Command twentyonePointAutoNo2nd(){
        
        var singlePath = PathPlanner.loadPath("21 Point No 2nd", new PathConstraints(2, 2));
        return Commands.sequence(
            m_intakeS.retractC(),

            m_drivebaseS.resetPoseToBeginningC(singlePath),
            m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3),
            m_intakeS.outtakeC().withTimeout(0.4),
            Commands.parallel(
                m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(singlePath)
            ),
            m_drivebaseS.chargeStationFrontFirstC()
        );
    }

    public Command twentyonePointAutoWith2nd(){

        var pathGroup = PathPlanner.loadPathGroup("21 Point With 2nd", new PathConstraints(2, 2),  new PathConstraints(0.5, 0.5), new PathConstraints(2, 2));
        return Commands.sequence(
            m_intakeS.retractC(),

            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3),
            m_intakeS.outtakeC().withTimeout(0.4),

            Commands.parallel(
                m_armS.goToPositionC(ArmConstants.OVERTOP_CONE_INTAKE_POSITION).asProxy().withTimeout(5),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
            ),

            //m_armS.goToPositionC(ArmConstants.OVERTOP_CONE_INTAKE_POSITION).asProxy().withTimeout(3),
            Commands.deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(1)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds()))),
                m_intakeS.intakeUntilBeamBreakC()
            ),

            Commands.parallel(
                m_intakeS.intakeC().withTimeout(0.1),
                m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
            ),

            m_drivebaseS.chargeStationFrontFirstC()
        );
    }

    public Command twentysevenPointAuto(){

        var pathGroup = PathPlanner.loadPathGroup("27 Point", new PathConstraints(2, 2),  new PathConstraints(0.5, 0.5), new PathConstraints(2, 2));
        return Commands.sequence(
            m_intakeS.retractC(),

            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3),
            m_intakeS.outtakeC().withTimeout(0.4),

            Commands.parallel(
                m_armS.goToPositionC(ArmConstants.OVERTOP_CONE_INTAKE_POSITION).asProxy().withTimeout(5),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
            ),

            Commands.parallel(
                m_intakeS.intakeUntilBeamBreakC().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(1)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
            ),


            Commands.parallel(
                m_intakeS.intakeC().withTimeout(0.1),
                m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
            ),

            Commands.parallel(
                m_intakeS.intakeC().withTimeout(0.1),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3)
            ),

            m_intakeS.outtakeC().withTimeout(0.4),

            Commands.parallel(
                m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(3)).andThen(m_drivebaseS.runOnce(()->m_drivebaseS.drive(new ChassisSpeeds())))
            ),

            m_drivebaseS.chargeStationBatteryFirstC()

        );
    }
}
