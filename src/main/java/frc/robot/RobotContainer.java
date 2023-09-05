package frc.robot;
import java.io.Console;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.POIManager.POIS;
import frc.robot.commands.arm.GoToPositionC;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.ArmS.ArmPosition;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.util.Alert;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;
import frc.robot.util.sparkmax.SparkMax;
import io.github.oblarg.oblog.annotations.Log;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */
    private final CommandXboxController m_driverController = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    private final CommandOperatorKeypad m_keypad;
    private final DrivebaseS m_drivebaseS;
    
    @Log
    private double lightSpeed = 0;

    @Log
    private double loopTime = 0;
    private final IntakeS m_intakeS = new IntakeS();
    private final ArmS m_armS;

    @Log
    private final Field2d m_field = new Field2d();
    // @Log
    // private final Field3d m_field3d = new Field3d();
    private final FieldObject2d m_target = m_field.getObject("target");

    private ArmPosition m_targetArmPosition = ArmConstants.STOW_POSITION;
    private Pose2d m_targetAlignmentPose = new Pose2d(1.909, 1.072, Rotation2d.fromRadians(Math.PI));
    private boolean m_isCubeSelected = true;

    
    private InputAxis m_fwdXAxis = new InputAxis("Forward", m_driverController::getLeftY)
        .withDeadband(0.2)
        .withInvert(true)
        .withSlewRate(3)
        .withSquaring(true);
    private InputAxis m_fwdYAxis = new InputAxis("Strafe", m_driverController::getLeftX)
        .withDeadband(0.2)
        .withInvert(true)
        .withSlewRate(3)
        .withSquaring(true);
    private InputAxis m_rotAxis = new InputAxis("Rotate", m_driverController::getRightX)
        .withDeadband(0.2)
        .withInvert(true)
        .withSlewRate(3);
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

    
    /**
     * Trigger that determines whether the drivebase is close enough to its target pose to score a cube.
     */
    //@Log(methodName = "getAsBoolean")
    private Trigger m_alignSafeToPlace;
    private boolean m_setupDone = false;
    
    public RobotContainer(Consumer<Runnable> addPeriodic) {
        if (RobotBase.isSimulation()) {
            CameraServer.startAutomaticCapture();
            PhotonCamera.setVersionCheckEnabled(false);
        }
        addPeriodic.accept(Alert::periodic);
        Timer.delay(0.1);
        m_drivebaseS = new DrivebaseS(addPeriodic);
        m_alignSafeToPlace = new Trigger(()->{
            Transform2d error = new Transform2d(m_targetAlignmentPose, m_drivebaseS.getPose());
            if (m_isCubeSelected) {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(3) &&
                Math.abs(error.getX()) < 0.3 &&
                Math.abs(error.getY()) < 0.2;
            } else {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(3) &&
                Math.abs(error.getX()) < 0.3 &&
                Math.abs(error.getY()) < 0.2;
            }
        });
    
        Timer.delay(0.1);
        m_armS = new ArmS(addPeriodic);

        /**
         * Set driver mode on the USB camera streamed through PhotonVision
         */
        PhotonCamera usbCam = new PhotonCamera("HD_USB_Camera");
        usbCam.setDriverMode(true);


        // Create the keypad with the functions it needs to set desired scoring action.
        m_keypad = new CommandOperatorKeypad(2, (pose)->{
            m_targetAlignmentPose = pose;
            m_field.getObject("Selection").setPose(pose);
        }, (position)->{m_targetArmPosition = position;},
        (selectedCube)->{m_isCubeSelected = selectedCube; m_intakeS.setGamePiece(m_isCubeSelected);});
        // Set scoring to right hybrid node.
        m_keypad.setpointCommand(0, 0).schedule();
        
        m_drivebaseS.setDefaultCommand(
            m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis)
        );
        // face downfield while Start is held
        m_driverController.start().whileTrue(
            m_drivebaseS.manualHeadingDriveC(m_fwdXAxis, m_fwdYAxis, ()->0)
        );

        configureButtonBindings();


        //Autonomous Option Selections:
        m_autoSelector.setDefaultOption("Cone Bal.-Bump Side", eighteenPointAuto(3));
        m_autoSelector.addOption("Cone Bal.-HP Side", eighteenPointAuto(5));
        //No Bump:
        m_autoSelector.addOption("2 Cone", fifteenPointAuto());
        m_autoSelector.addOption("2 Cone Bal.", twentysevenPointAuto());
        // m_autoSelector.addOption("Cone Over+Back - HP", overBackAuto(5));
        // m_autoSelector.addOption("Cone Over+Back - Bump", overBackAuto(3));
        m_autoSelector.addOption("2 Cone Over Bump", bumpTwoConeAuto());

        // m_autoSelector.addOption("Cone+Over Bump [UNTESTED]", ninePointAuto());

        m_autoSelector.addOption("Do Nothing", Commands.none());
        // m_autoSelector.addOption("21 Point No 2nd", twentyonePointAutoNo2nd());
        // m_autoSelector.addOption("21 Point With 2nd", twentyonePointAutoWith2nd());
        // m_autoSelector.addOption("27 Point (Cone Cube)", twentysevenPointConeCubeAuto());
        // m_autoSelector.addOption("27 Point (2 Cone)", twentysevenPointConeConeAuto());


        m_field.getObject("bluePoses").setPoses(POIManager.BLUE_COMMUNITY);
        m_field.getObject("redPoses").setPoses(POIManager.RED_COMMUNITY);
        SmartDashboard.putData(m_autoSelector);
        Timer.delay(0.2);
        SparkMax.burnFlashInSync();
        Timer.delay(0.2);
        m_setupDone = true;
        addPeriodic.accept(()->{
            if (DriverStation.isDisabled() && m_setupDone) {
                LightStripS.getInstance().requestState(States.SetupDone);
            }
        });
    }

    /**
     * Command factory for 
     */
    private Command alignToSelectedScoring() {
        return  m_drivebaseS.chasePoseC(
            ()->m_targetAlignmentPose.transformBy(m_isCubeSelected ? new Transform2d() : m_intakeS.getConeCenterOffset()
            )
        );
    }

    public void configureButtonBindings() {
        // Align, score, and stow.
        m_driverController.a().toggleOnTrue(
                alignToSelectedScoring().asProxy()
                .until(
                    ()->
                    m_isCubeSelected ? m_alignSafeToPlace.getAsBoolean() : false)
                .andThen(autoScoreSequenceCG().asProxy())
           
            );
        // Score and stow.
        m_driverController.b().toggleOnTrue(
            Commands.sequence(
                m_intakeS.outtakeC().withTimeout(0.5),
                m_armS.stowIndefiniteC()
            )
            );
        // OFFICIAL CALEB PREFERENCE
        m_driverController.y().toggleOnTrue(armIntakeSelectedCG(
            ArmConstants.OVERTOP_CUBE_INTAKE_POSITION,
            ArmConstants.OVERTOP_CONE_INTAKE_POSITION,
            ()->m_isCubeSelected
        ));
        m_driverController.x().onTrue(m_armS.stowIndefiniteC());
        m_driverController.back().onTrue(m_intakeS.runOnce(m_intakeS::toggle));


        m_driverController.rightBumper().toggleOnTrue(armIntakeSelectedCG(
            ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT, 
            ArmConstants.RAMP_CONE_INTAKE_POSITION, ()->m_isCubeSelected));

        m_driverController.rightTrigger().toggleOnTrue(armIntakeSelectedCG(
            ArmConstants.ArmPositions.FRONT_GROUND_CUBE, 
            ArmConstants.GROUND_CONE_INTAKE_POSITION, ()->m_isCubeSelected));
            
        m_driverController.leftBumper().toggleOnTrue(armIntakeSelectedCG(
            ArmConstants.PLATFORM_CUBE_INTAKE_POSITION, 
            ArmConstants.PLATFORM_CONE_INTAKE_POSITION, ()->m_isCubeSelected));
        m_driverController.leftTrigger().onTrue(m_armS.goToPositionIndefiniteC(ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT));
        // m_driverController.leftTrigger().whileTrue(new ConditionalCommand(
        //     m_drivebaseS.chasePoseC(()->POIManager.ownPOI(POIS.CUBE_RAMP)),
        //     m_drivebaseS.chasePoseC(()->POIManager.ownPOI(POIS.CONE_RAMP)),
        //     ()->m_isCubeSelected));

        // Button on keypad for pulsing the intake in.
        m_keypad.action().onTrue(m_intakeS.intakeC().withTimeout(0.25));
        m_keypad.enter().toggleOnTrue(
            autoScoreSequenceCG()
        );
    
        // D-pad driving slowly relative to alliance wall.
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
        //System.out.println(m_autoSelector.getSelected().getRequirements().toString());
        //lightSpeed = LightStripS.getInstance().getSpeed();
        TimingTracer.update();
        loopTime = TimingTracer.getLoopTime();
        SmartDashboard.putNumber("loopTime", loopTime);

        //SmartDashboard.putNumber("loopTime", TimingTracer.getLoopTime());
        LightStripS.getInstance().requestState(m_isCubeSelected? States.RequestingCube : States.RequestingCone);
        /* Trace the loop duration and plot to shuffleboard */
        LightStripS.getInstance().periodic();
        m_drivebaseS.drawRobotOnField(m_field);
        m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
        ///]m_field3d.setRobotPose(new Pose3d(m_drivebaseS.getPose().getX(), m_drivebaseS.getPose().getY(), 0, m_drivebaseS.getRotation3d()));
    }

    public void onEnabled(){
        m_drivebaseS.resetRelativeRotationEncoders();
        m_armS.resetAllControllers();
    }
    public void onDisabled() {
    }

    /**
     * Command factory for intaking a game piece.
     * @param position The arm position
     * @param isCube Whether the intake should be in cube mode or cone mode.
     * @return
     */
    public Command armIntakeCG(ArmPosition position, boolean isCube) {
        return 
        Commands.sequence(
            // Start intaking, and stop when a piece is detected.
            Commands.deadline(
                m_intakeS.setGamePieceC(()->isCube).andThen(m_intakeS.intakeUntilBeamBreakC()).asProxy(),
                // move to arm position while intaking.
                m_armS.goToPositionIndefiniteC(position)

            ),
            Commands.parallel(
                // Wait a bit, then pulse the intake to ensure piece collection.
                Commands.waitSeconds(0.75).andThen(m_intakeS.intakeC().withTimeout(0.75)).asProxy(),
                // stow the arm
                m_armS.stowIndefiniteC(), 
                Commands.run(()->LightStripS.getInstance().requestState(isCube ? States.IntakedCube : States.IntakedCone)).asProxy().withTimeout(0.75)
            )
        );
    }

    public Command armIntakeSelectedCG(ArmPosition cubePosition, ArmPosition conePosition, BooleanSupplier isCube) {
        return Commands.either(
            armIntakeCG(cubePosition, true), armIntakeCG(conePosition, false), isCube);
    }
    public Command autoScoreSequenceCG() {
        return sequence(
                m_armS.goToPositionC(()->m_targetArmPosition),
                        m_intakeS.outtakeC().withTimeout(0.4),
                        m_armS.stowC()
            )
            .deadlineWith(Commands.run(()->LightStripS.getInstance().requestState(States.Scoring)));

    }

    //Autonomous Commands:
    

    public Command eighteenPointAuto(int blueColumn){
        
        return Commands.sequence(
            m_intakeS.setGamePieceC(()->false).asProxy(),

            Commands.runOnce(
                ()->m_drivebaseS.resetPose(
                    NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(blueColumn), AllianceWrapper.getAlliance())
                )),
            
            m_keypad.blueSetpointCommand(blueColumn, 2),
            Commands.deadline(
                Commands.sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3).asProxy(),
                    m_intakeS.outtakeC().withTimeout(0.4).asProxy()
                ),
                alignToSelectedScoring().asProxy()
            ),

            m_armS.goToPositionC(ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT).asProxy(),
            m_drivebaseS.chargeStationAlignC().asProxy(),
            m_drivebaseS.run(()->m_drivebaseS.drive(new ChassisSpeeds(0, 0, 0.1))).withTimeout(0.5).asProxy()
            //m_drivebaseS.chargeStationBatteryFirstC()
        ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()));
    }

    // public Command overBackAuto(int blueColumn) {
    //     return Commands.sequence(
    //         m_intakeS.retractC().asProxy(),

    //         Commands.runOnce(
    //             ()->m_drivebaseS.resetPose(
    //                 NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(blueColumn), AllianceWrapper.getAlliance())
    //             )),
            
    //         m_keypad.blueSetpointCommand(blueColumn, 2),
    //         Commands.deadline(
    //             Commands.sequence(
    //                 m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3).asProxy(),
    //                 m_intakeS.outtakeC().withTimeout(0.4).asProxy()
    //             ),
    //             alignToSelectedScoring().asProxy()
    //         ),

    //         m_armS.goToPositionC(ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT).asProxy(),
    //         sequence(
    //             m_drivebaseS.run(()->m_drivebaseS.driveAllianceRelative(new ChassisSpeeds(3, 0, 0))).withTimeout(5).until(()->POIManager.mirrorPoseAlliance(m_drivebaseS.getPose()).getX() > 6).asProxy(),
    //             m_drivebaseS.run(()->m_drivebaseS.driveAllianceRelative(new ChassisSpeeds(3, 0, 0))).withTimeout(3).until(()->Math.abs(m_drivebaseS.getPitch()) > Units.degreesToRadians(10)).asProxy(),
    //             m_drivebaseS.chargeStationAlignC().asProxy()
    //         ),
    //         m_drivebaseS.run(()->m_drivebaseS.drive(new ChassisSpeeds(0, 0, 0.1))).withTimeout(0.5).asProxy()
    //         //m_drivebaseS.chargeStationBatteryFirstC()
    //     ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()));
    // }

    public Command ninePointAuto(){
        
        return Commands.sequence(
            m_intakeS.setGamePieceC(()->false).asProxy(),

            Commands.runOnce(
                ()->m_drivebaseS.resetPose(
                    NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(0), AllianceWrapper.getAlliance())
                )),
            
            m_keypad.blueSetpointCommand(0, 2),
            Commands.deadline(
                Commands.sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3).asProxy(),
                    m_intakeS.outtakeC().withTimeout(0.4).asProxy()
                ),
                alignToSelectedScoring().asProxy()
            ),
            m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy(),
            m_drivebaseS.chasePoseC(
                ()->NomadMathUtil.mirrorPose(new Pose2d(6, 0.87, Rotation2d.fromRadians(Math.PI)), AllianceWrapper.getAlliance()))
            .asProxy()
            //m_drivebaseS.chargeStationBatteryFirstC()
        ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()));
    }

    //No Bump

    private Command twoConeAuto() {
        var pathGroup = PathPlanner.loadPathGroup("27 Point", new PathConstraints(2, 2), new PathConstraints(4, 3));
        return Commands.sequence(
            m_intakeS.setGamePieceC(()->false).asProxy(),
            m_keypad.blueSetpointCommand(8, 2),
            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            Commands.deadline(
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3),
                alignToSelectedScoring().asProxy()
            ),
            m_intakeS.outtakeC().withTimeout(0.3).asProxy(),
            
            m_keypad.blueSetpointCommand(6, 2),

            Commands.deadline(
                Commands.race(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(0)).asProxy(),
                    m_intakeS.intakeUntilBeamBreakC(m_intakeS.autoStagedIntakeC()).asProxy()
                ),
                m_armS.goToPositionC(ArmConstants.OVERTOP_CONE_INTAKE_POSITION).asProxy().withTimeout(5)  
            ),
            Commands.parallel(
                
                m_intakeS.intakeC().withTimeout(0.5).asProxy(),
                m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(1)).asProxy()
            ),
            Commands.deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy(),
                    m_intakeS.outtakeC().withTimeout(0.3).asProxy()
                ),
                alignToSelectedScoring().asProxy()
            )

        );
    }


    private Command bumpTwoConeAuto() {
        var pathGroup = PathPlanner.loadPathGroup("Bump 27 Point", new PathConstraints(1.5, 2));
        return Commands.sequence(
            m_intakeS.setGamePieceC(()->false).asProxy(),
            m_keypad.blueSetpointCommand(0, 2),
            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            Commands.deadline(
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy().withTimeout(3),
                alignToSelectedScoring().asProxy()
            ),
            m_intakeS.outtakeC().withTimeout(0.3).asProxy(),
            m_armS.goToPositionC(ArmConstants.RETRACTED_SCORE_CONE_POSITION).asProxy(),
            m_keypad.blueSetpointCommand(2, 2),

            Commands.deadline(
                Commands.race(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(0)).andThen(m_drivebaseS.stopOnceC()).asProxy(),
                    m_intakeS.intakeUntilBeamBreakC(m_intakeS.autoStagedIntakeC()).asProxy()
                ),
                m_armS.goToPositionC(ArmConstants.OVERTOP_CONE_INTAKE_POSITION).asProxy().withTimeout(5)  
            ),
            Commands.parallel(
                
                m_intakeS.intakeC().withTimeout(0.1).asProxy(),
                m_armS.goToPositionC(ArmConstants.RETRACTED_SCORE_CONE_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(pathGroup.get(1)).asProxy()
            ),
            Commands.deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).asProxy(),
                    m_intakeS.outtakeC().withTimeout(0.3).asProxy(),
                    m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy()
                ),
                alignToSelectedScoring().asProxy()
            )

        );
    }




    public Command fifteenPointAuto(){
        var backOutPath = PathPlanner.loadPath("15 Point Back Out", new PathConstraints(4, 3));
        
        return twoConeAuto().andThen(
            Commands.parallel(
                m_keypad.blueSetpointCommand(7, 2),
                m_armS.goToPositionC(ArmConstants.STOW_POSITION).asProxy().withTimeout(3),
                m_drivebaseS.pathPlannerCommand(backOutPath).asProxy()
            )
            
        );
    }


    public Command twentysevenPointAuto(){
        var pathGroup = PathPlanner.loadPathGroup("27 Point", new PathConstraints(2, 2));
        return twoConeAuto().andThen(

            Commands.parallel(
                m_armS.goToPositionC(ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT).asProxy().withTimeout(3),
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).asProxy(),
                    m_drivebaseS.chargeStationAlignC().asProxy(),
                    m_drivebaseS.run(()->m_drivebaseS.drive(new ChassisSpeeds(0, 0, 0.1))).withTimeout(0.5).asProxy()
                ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()))
                
            )
        );
    }
}
