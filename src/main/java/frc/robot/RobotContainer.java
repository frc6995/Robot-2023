package frc.robot;
import java.io.Console;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import autolog.AutoLog;
import autolog.Logged;
import autolog.AutoLog.BothLog;
import autolog.AutoLog.BothLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.POIManager.POIS;
import frc.robot.commands.arm.GoToPositionC;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.driver.CommandOperatorKeypad.Button;
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

public class RobotContainer implements Logged{

    /**
     * Establishes the controls and subsystems of the robot
     */
    private final CommandXboxController m_driverController = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    private final CommandOperatorKeypad m_keypad;
    private final DrivebaseS m_drivebaseS;
    
    @BothLog
    private double lightSpeed = 0;

    @BothLog
    private double loopTime = 0;
    private LinearFilter loopTimeAverage = LinearFilter.movingAverage(1);
    private final IntakeS m_intakeS = new IntakeS();
    private final ArmS m_armS;

    @BothLog
    private final Field2d m_field = new Field2d();
    @BothLog(path="/DriverDisplay/field", level = 1)
    private final Field2d m_driverField = new Field2d();
    @BothLog(path="/DriverDisplay/alliance", level=1)
    public boolean isBlueAlliance() {
        return AllianceWrapper.isBlue();
    }
    // @Log
    // private final Field3d m_field3d = new Field3d();
    private final FieldObject2d m_target = m_field.getObject("target");

    private BooleanPublisher enabledEntry = NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/enabled").publish();
    private BooleanPublisher controller1Entry = NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/controller1").publish();
    private BooleanPublisher controller2Entry = NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/controller2").publish();

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
        .withSlewRate(1, -6);
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

    
    /**
     * Trigger that determines whether the drivebase is close enough to its target pose to score a cube.
     */
    private Trigger m_alignSafeToPlace;
    private Trigger m_alignSafeToPremove;
    private boolean m_setupDone = false;
    
    @BothLog
    private double getFwdAxis() {
        return m_fwdXAxis.getAsDouble();
    }
    @BothLog
    private double getSideAxis() {
        return m_fwdYAxis.getAsDouble();
    }
    @BothLog
    private double getRotAxis() {
        return m_rotAxis.getAsDouble();
    }

    public RobotContainer(Consumer<Runnable> addPeriodic) {
        if (RobotBase.isSimulation()) {
            CameraServer.startAutomaticCapture();
            PhotonCamera.setVersionCheckEnabled(false);
        }
        addPeriodic.accept(Alert::periodic);
        Timer.delay(0.1);
        m_drivebaseS = new DrivebaseS(addPeriodic, (name, traj)->{m_field.getObject(name).setTrajectory(traj);});
        m_keypad = new CommandOperatorKeypad(2);
        m_alignSafeToPlace = new Trigger(()->{
            Transform2d error = new Transform2d(
                getTargetAlignmentPose(), m_drivebaseS.getPose());
            if (isCubeSelected()) {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(3) &&
                Math.abs(error.getX()) < 0.3 &&
                Math.abs(error.getY()) < 0.2;
            } else {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(2) &&
                Math.abs(error.getX()) < 0.02 &&
                Math.abs(error.getY()) < Units.inchesToMeters(0.5);
            }
        });
        m_alignSafeToPremove = new Trigger(()->{
            Transform2d error = new Transform2d(
                getTargetAlignmentPose(), m_drivebaseS.getPose());
            if (isCubeSelected()) {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(10) &&
                Math.abs(error.getX()) < 0.5 &&
                Math.abs(error.getY()) < 0.5;
            } else {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(10) &&
                Math.abs(error.getX()) < 0.5 &&
                Math.abs(error.getY()) < 0.5;
            }
        });
    
        Timer.delay(0.1);
        m_armS = new ArmS(addPeriodic, m_intakeS::hitBeamBreak);





        configureButtonBindings();
        addAutoRoutines();

        m_field.getObject("bluePoses").setPoses(POIManager.BLUE_COMMUNITY);
        m_field.getObject("redPoses").setPoses(POIManager.RED_COMMUNITY);
        SmartDashboard.putData(m_autoSelector);
        AutoLog.setupLogging(this,"Robot", true);
        DataLogManager.logNetworkTables(false);
        Timer.delay(0.3);
        SparkMax.burnFlashInSync();
        Timer.delay(0.2);
        Commands.sequence(
            waitSeconds(4),
            runOnce(()->m_setupDone = true)
        ).ignoringDisable(true).schedule();
        DriverStation.reportWarning("Setup Done", false);
    }

    private Pose2d getTargetAlignmentPose() {
        return POIManager.ownCommunity().get(
            (int) m_keypad.get() % 9
        ).transformBy(isCubeSelected() ? new Transform2d() : m_intakeS.getConeCenterOffset());
    }
    private ArmPosition getTargetArmPosition() {
        double node = m_keypad.get();
        if (node <= 8) {
            return ArmConstants.SCORE_HYBRID_POSITION;
        } else if (node <= 17) { // mid
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_MID_CONE_POSITION;
            }
        } else { // high
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_HIGH_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_HIGH_CONE_POSITION;
            }
        }
    }
    private ArmPosition getPrescoreArmPosition() {
        double node = m_keypad.get();
        if (node <= 8) {
            return ArmConstants.SCORE_HYBRID_POSITION;
        } else if (node <= 17) { // mid
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_MID_CONE_POSITION;
            }
        } else { // high
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_HIGH_CONE_POSITION;
            }
        }
    }
    private boolean isCubeSelected() {
        double node = m_keypad.get();
        return (node <= 8) || (node > 8 && node % 3 == 1);
    }
    private boolean isHybridSelected() {
        double node = m_keypad.get();
        return (node <= 8);
    }
    /**
     * Command factory for 
     */
    private Command alignToSelectedScoring() {
        return  m_drivebaseS.chasePoseC(
            this::getTargetAlignmentPose
            );
    }

    public void configureButtonBindings() {

        // Create the keypad with the functions it needs to set desired scoring action.

        // Set scoring to right hybrid node.
        m_keypad.setpointCommand(0, 0).schedule();
        
        m_drivebaseS.setDefaultCommand(
            m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis)
        );
        // face downfield while Start is held
        // Align, score, and stow.
        m_driverController.a().toggleOnTrue(
            sequence(
                deadline(
                    sequence(
                        sequence(
                             waitUntil(m_alignSafeToPremove),
                            m_armS.goToPositionC(this::getTargetArmPosition),
                            waitUntil(m_alignSafeToPlace)
                        )//.until(m_alignSafeToPlace)//,
                        //m_armS.goToPositionC(this::getTargetArmPosition)
                    ),
                    alignToSelectedScoring().asProxy()
                ).until(m_keypad.leftGrid().and(m_keypad.centerGrid()).and(m_keypad.rightGrid())),
                m_intakeS.outtakeC(this::isCubeSelected).withTimeout(0.4),
                m_armS.stowC() 
            )
        );
        // Score and stow.
        m_driverController.b().toggleOnTrue(m_armS.stowIndefiniteC());
        m_driverController.back().and(DriverStation::isDisabled).onTrue(Commands.runOnce(m_armS::markWristHomed).ignoringDisable(true));

        m_driverController.rightBumper().toggleOnTrue(armIntakeCG(
            ArmPositions.FRONT_UP_FLOOR, false));

        m_driverController.rightTrigger().toggleOnTrue(armIntakeCG(
            ArmPositions.BACK_TIPPED_FLOOR, 
            ArmPositions.BACK_TIPPED_FLOOR_PRESTOW,
            false));
            
        m_driverController.leftBumper().toggleOnTrue(armIntakeCG(
            ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT, new ArmPosition(ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT.pivotRadians, ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT.armLength, Units.degreesToRadians(-40)), false));
        m_driverController.leftTrigger().onTrue(armIntakeCG(ArmConstants.GROUND_CUBE_INTAKE_POSITION, true));

        m_driverController.x().whileTrue(m_drivebaseS.chasePickupC(()-> POIManager.ownPOI(AllianceWrapper.isRed() ? POIS.GRID_PLAT : POIS.WALL_PLAT)));
        m_driverController.y().whileTrue(m_drivebaseS.chasePickupC(()-> POIManager.ownPOI(AllianceWrapper.isRed() ? POIS.WALL_PLAT : POIS.GRID_PLAT)));
        // m_driverController.x().onTrue(armIntakeCG(
        //     ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT, false));
        // m_driverController.y().onTrue(armIntakeCG(
        //     ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT, false));

        // Button on keypad for pulsing the intake in.
        m_keypad.action().onTrue(m_intakeS.intakeC(this::isCubeSelected).withTimeout(0.5));
        m_keypad.enter().toggleOnTrue(
            autoScoreSequenceCG()
        );

        // D-pad driving slowly relative to intake direction.
        m_driverController.povCenter().negate().whileTrue(m_drivebaseS.run(()->{
                double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
                if (m_armS.getArmPosition().pivotRadians > Math.PI) {
                    pov += Math.PI;
                }
                double adjustSpeed = 0.75; // m/s
                m_drivebaseS.drive(
                    new ChassisSpeeds(
                        Math.cos(pov) * adjustSpeed,
                        Math.sin(pov) * adjustSpeed,
                        m_rotAxis.getAsDouble() * DriveConstants.MAX_TURN_SPEED
                    )
                );
            }
        ));        
    }
    public void addAutoRoutines() {
        m_autoSelector.setDefaultOption("Do Nothing", Commands.none());
        m_autoSelector.addOption("3pc HP", highConeHighCubeHPSide().andThen(midCubeHPAddon()));
        m_autoSelector.addOption("3pc Bump", highConeHighCubeBumpSide().andThen(midCubeBumpAddon()));
        m_autoSelector.addOption("2.5pc Bump Climb", highConeHighCubeBumpSide().andThen(cubePickupClimbBumpAddon()));

        m_autoSelector.addOption("BumpSide 1 Cone Bal", eighteenPointAuto(3));
        m_autoSelector.addOption("HP Side 1 Cone Bal", eighteenPointAuto(5));
    }


    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }

    @BothLog
    public boolean getSafeToPlace() {
        return m_alignSafeToPlace.getAsBoolean();
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            enabledEntry.set(false);
            if (m_setupDone) {
                LightStripS.getInstance().requestState(States.SetupDone);
            }
            else {
                LightStripS.getInstance().requestState(States.Disabled);
            }

        } else {
            enabledEntry.set(true);
        }
        controller1Entry.set(DriverStation.isJoystickConnected(0));
        controller2Entry.set(DriverStation.isJoystickConnected(2));
        //var armPosition = m_armS.getArmPosition();
        //m_intakeS.setHandCamFlipped(armPosition.pivotRadians + armPosition.wristRadians > Math.PI/2);
        //System.out.println(m_autoSelector.getSelected().getRequirements().toString());
        //lightSpeed = LightStripS.getInstance().getSpeed();
        TimingTracer.update();
        loopTime = loopTimeAverage.calculate(TimingTracer.getLoopTime());
        SmartDashboard.putNumber("loopTime", loopTime);

        // //SmartDashboard.putNumber("loopTime", TimingTracer.getLoopTime());
        LightStripS.getInstance().requestState(isCubeSelected()? States.RequestingCube : States.RequestingCone);
        // /* Trace the loop duration and plot to shuffleboard */
        LightStripS.getInstance().periodic();
        m_drivebaseS.drawRobotOnField(m_field);
        m_driverField.setRobotPose(m_drivebaseS.getPose());
        m_driverField.getObject("target").setPose(getTargetAlignmentPose());
        m_field.getObject("selection").setPose(getTargetAlignmentPose());
        m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
        AutoLog.update();
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
    public Command armIntakeCG(ArmPosition position, ArmPosition prestow, boolean isCube) {
        return 
        Commands.sequence(
            // Start intaking, and stop when a piece is detected.\
            Commands.deadline(
                Commands.waitSeconds(0.3).andThen(m_intakeS.intakeUntilBeamBreakC(isCube)).andThen(m_intakeS.intakeC(()->isCube).withTimeout(0.2)).asProxy(),
                // move to arm position while intaking.
                m_armS.goToPositionIndefiniteC(position)

            ),
            Commands.parallel(
                // Wait a bit, then pulse the intake to ensure piece collection.
                Commands.waitSeconds(0.75).andThen(m_intakeS.intakeC(()->isCube).withTimeout(0.75)).asProxy(),
                // stow the arm
                m_armS.goToPositionC(()->prestow).andThen( m_armS.goToPositionC(()->isCube? ArmPositions.CUBE_STOW : ArmPositions.STOW)),
                Commands.run(()->LightStripS.getInstance().requestState(isCube ? States.IntakedCube : States.IntakedCone)).asProxy().withTimeout(0.75)
            )
        );
    }

    public Command armIntakeCG(ArmPosition position, boolean isCube) {
        return armIntakeCG(position, isCube? ArmPositions.CUBE_STOW : ArmPositions.STOW, isCube);
    }

    public Command armIntakeSelectedCG(ArmPosition cubePosition, ArmPosition conePosition, BooleanSupplier isCube) {
        return Commands.either(
            armIntakeCG(cubePosition, true), armIntakeCG(conePosition, false), isCube);
    }
    public Command autoScoreSequenceCG() {
        return sequence(
                m_armS.goToPositionC(this::getTargetArmPosition).until(m_keypad.leftGrid().and(m_keypad.centerGrid()).and(m_keypad.rightGrid())),
                        m_intakeS.outtakeC(this::isCubeSelected, this::isHybridSelected).withTimeout(0.4),
                        m_armS.stowC()
            )
            .deadlineWith(Commands.run(()->LightStripS.getInstance().requestState(States.Scoring)));

    }

    public Command driveAlignPlat(Supplier<POIS> target) {
        return m_drivebaseS.run(()->{
            double downfield = (AllianceWrapper.getAlliance() == Alliance.Red) ? 
                    Math.PI : 0.0;
            double rot = m_drivebaseS.m_thetaController.calculate(
                m_drivebaseS.getPoseHeading().getRadians(), downfield);
            m_drivebaseS.driveFieldRelative(
            new ChassisSpeeds(
                m_fwdXAxis.getAsDouble() * Constants.DriveConstants.MAX_LINEAR_SPEED / 2 * Math.cos(downfield),
                MathUtil.clamp(
                m_drivebaseS.m_yController.calculate(
                    m_drivebaseS.getPose().getY(),
                    POIManager.ownPOI(target.get()).getY() + 0.25 * m_rotAxis.getAsDouble() * Math.cos(downfield)),
                -1, 1),
                rot
            ));
        });
    }

    //Autonomous Commands:
    
    // region oldAutos
    public Command eighteenPointAuto(int blueColumn){
        
        return Commands.sequence(
            Commands.runOnce(
                ()->m_drivebaseS.resetPose(
                    NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(blueColumn), AllianceWrapper.getAlliance())
                )),
            
            m_keypad.blueSetpointCommand(blueColumn, 2),
            Commands.deadline(
                Commands.sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4)
                ),
                alignToSelectedScoring()
            ),

            m_armS.goToPositionC(ArmConstants.CLIMBING_POSITION),
            m_drivebaseS.chargeStationAlignC(),
            m_drivebaseS.xLockC()
            //m_drivebaseS.chargeStationBatteryFirstC()
        ).finallyDo((end)->m_drivebaseS.drive(new ChassisSpeeds()));
    }

    public Command ninePointAuto(){
        
        return Commands.sequence(
            Commands.runOnce(
                ()->m_drivebaseS.resetPose(
                    NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(0), AllianceWrapper.getAlliance())
                )),
            
            m_keypad.blueSetpointCommand(0, 2),
            Commands.deadline(
                Commands.sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3).asProxy(),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4).asProxy()
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


    // region newAutos
    public Command highConeHighCubeHPSide() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            // Target the HP-side high cone
            m_keypad.blueSetpointCommand(8, 2),
            // Step 1: Align and score
            deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4)
                )//,
                // sequence(
                //     alignToSelectedScoring().until(m_alignSafeToPlace)
                //     .andThen(m_drivebaseS.stopC())
                // )
            ),
            // Step 2: Fetch cube 1
            m_keypad.blueSetpointCommand(7, 2),
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)),
                m_intakeS.intakeC(()->true).until(m_intakeS::hitBeamBreak),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
                
            ).withTimeout(4),
            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(1)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            m_intakeS.outtakeC(()->true).withTimeout(0.4)
         );
    }
    public Command midCubeHPAddon() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
            m_keypad.blueSetpointCommand(7, 1),
            // head back out
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.stopOnceC()),
                Commands.waitSeconds(0.5).andThen(m_intakeS.intakeC(()->true).until(m_intakeS::hitBeamBreak)),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
               
            ).withTimeout(4),

            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(3)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_MID_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            //m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
            // extend, score
            m_intakeS.run(()->m_intakeS.outtakeCube(6)).withTimeout(0.4)
        );
    }

    public Command highConeHighCubeBumpSide() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
            m_drivebaseS.resetPoseToBeginningC(pathGroup.get(0)),
            // Target the HP-side high cone
            m_keypad.blueSetpointCommand(0, 2),
            // Step 1: Align and score
            deadline(
                sequence(
                    m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION),
                    m_intakeS.outtakeC(()->false).withTimeout(0.4)
                )//,
                // sequence(
                //     alignToSelectedScoring().until(m_alignSafeToPlace)
                //     .andThen(m_drivebaseS.stopC())
                // )
            ),
            // Step 2: Fetch cube 1
            m_keypad.blueSetpointCommand(1, 1),
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(0)),
                Commands.waitSeconds(0.5).andThen(m_intakeS.intakeC(()->true).until(m_intakeS::hitBeamBreak)),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
                
            ).withTimeout(4),
            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(1)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            m_intakeS.outtakeC(()->true).withTimeout(0.4)
         );
    }

    public Command midCubeBumpAddon() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return sequence(
        m_keypad.blueSetpointCommand(1, 1),
            // head back out
            deadline(
                m_drivebaseS.pathPlannerCommand(pathGroup.get(2)).andThen(m_drivebaseS.stopOnceC()),
                Commands.waitSeconds(0.5).andThen(m_intakeS.intakeC(()->true).until(m_intakeS::hitBeamBreak)),
                m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
                // drive from first cone score to cube
               
            ).withTimeout(4),

            // Drive back while stowing cube
            // when we're close enough to score, move on
            deadline(
                sequence(
                    m_drivebaseS.pathPlannerCommand(pathGroup.get(3)),
                    alignToSelectedScoring()
                ).until(m_alignSafeToPlace),
                m_armS.goToPositionC(ArmConstants.SCORE_MID_CUBE_POSITION),
                m_intakeS.run(()->m_intakeS.intakeCube(0.5))
            ),
            //m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CUBE_POSITION),
            // extend, score
            m_intakeS.run(()->m_intakeS.outtakeCube(6)).withTimeout(0.4)
         );
    }

    public Command cubePickupClimbBumpAddon() {
        var pathGroup = PathPlanner.loadPathGroup("High Cone High Cube Bump",
        new PathConstraints(2, 2),
        new PathConstraints(4, 2.5));
        return deadline(
            m_drivebaseS.pathPlannerCommand(pathGroup.get(4)),
            m_intakeS.intakeC(()->true).until(m_intakeS::acquiredCube),
            m_armS.goToPositionC(ArmConstants.GROUND_CUBE_INTAKE_POSITION)
        ).andThen(
            parallel(
                m_drivebaseS.chargeStationAlignC(),
                m_armS.goToPositionC(ArmPositions.CUBE_STOW)
            )
        );
    }

    // public Command overBackAuto(int blueColumn) {
    //     return Commands.sequence(
    //         m_keypad.blueSetpointCommand(blueColumn, 2),
    //         Commands.runOnce(
    //             ()->m_drivebaseS.resetPose(
    //                 NomadMathUtil.mirrorPose(POIManager.BLUE_COMMUNITY.get(blueColumn), AllianceWrapper.getAlliance())
    //             )),
            

    //         Commands.deadline(
    //             Commands.sequence(
    //                 m_armS.goToPositionC(ArmConstants.SCORE_HIGH_CONE_POSITION).withTimeout(3),
    //                 m_intakeS.outtakeC().withTimeout(0.4)
    //             ),
    //             alignToSelectedScoring()
    //         ),

    //         m_armS.goToPositionC(ArmConstants.RAMP_CUBE_INTAKE_POSITION_FRONT),
    //         sequence(
                
    //             m_drivebaseS.run(()->m_drivebaseS.driveAllianceRelative(new ChassisSpeeds(3, 0, 0))).withTimeout(5).until(()->POIManager.mirrorPoseAlliance(m_drivebaseS.getPose()).getX() > 6),
    //             m_drivebaseS.chargeStationAlignC()
    //         ),
    //         m_drivebaseS.xLockC()
    //     );
    //}
}
