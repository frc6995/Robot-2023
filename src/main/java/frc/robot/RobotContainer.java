package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import autolog.AutoLog;
import autolog.Logged;
import autolog.AutoLog.BothLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.POIManager.POIS;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.util.Alert;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.TimingTracer;
import frc.robot.util.sparkmax.SparkMax;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainer implements Logged {

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

    @BothLog(path = "/DriverDisplay/field", level = 1)
    private final Field2d m_driverField = new Field2d();

    @BothLog(path = "/DriverDisplay/alliance", level = 1)
    public boolean isBlueAlliance() {
        return AllianceWrapper.isBlue();
    }

    private final Autos m_autos;

    private BooleanPublisher enabledEntry = NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/enabled")
            .publish();
    private BooleanPublisher controller1Entry = NetworkTableInstance.getDefault()
            .getBooleanTopic("/DriverDisplay/controller1").publish();
    private BooleanPublisher controller2Entry = NetworkTableInstance.getDefault()
            .getBooleanTopic("/DriverDisplay/controller2").publish();

    private InputAxis m_fwdXAxis = new InputAxis("Forward", m_driverController::getLeftY)
            .withDeadband(0.1)
            .withInvert(true)
            .withSlewRate(3)
            .withSquaring(true);
    private InputAxis m_fwdYAxis = new InputAxis("Strafe", m_driverController::getLeftX)
            .withDeadband(0.1)
            .withInvert(true)
            .withSlewRate(3)
            .withSquaring(true);
    private InputAxis m_rotAxis = new InputAxis("Rotate", m_driverController::getRightX)
            .withDeadband(0.2)
            .withInvert(true)
            .withSlewRate(1.33, -6);
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

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
        m_drivebaseS = new DrivebaseS(addPeriodic, (name, traj) -> {
            m_field.getObject(name).setTrajectory(traj);
        });
        m_keypad = new CommandOperatorKeypad(2);

        Timer.delay(0.1);
        m_armS = new ArmS(addPeriodic, m_intakeS::hitBeamBreak);

        m_autos = new Autos(m_drivebaseS, m_armS, m_intakeS, m_keypad);
        configureButtonBindings();
        addAutoRoutines();

        m_field.getObject("bluePoses").setPoses(POIManager.BLUE_COMMUNITY);
        m_field.getObject("redPoses").setPoses(POIManager.RED_COMMUNITY);
        SmartDashboard.putData(m_autoSelector);
        AutoLog.setupLogging(this, "Robot", true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.logNetworkTables(false);
        Timer.delay(0.3);
        SparkMax.burnFlashInSync();
        Timer.delay(0.2);
        Commands.sequence(
                waitSeconds(4),
                runOnce(() -> m_setupDone = true)).ignoringDisable(true).schedule();
        DriverStation.reportWarning("Setup Done", false);
    }

    public void configureButtonBindings() {

        // Create the keypad with the functions it needs to set desired scoring action.

        // Set scoring to right hybrid node.
        m_keypad.setpointCommand(0, 0).schedule();

        m_drivebaseS.setDefaultCommand(m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis));
        // Align, score, and stow.
        m_driverController.a().toggleOnTrue(m_autos.alignScore());
        // Score and stow.
        m_driverController.b().toggleOnTrue(m_armS.stowIndefiniteC());
        m_driverController.back().and(DriverStation::isDisabled)
                .onTrue(Commands.runOnce(m_armS::markWristHomed).ignoringDisable(true));

        m_driverController.rightBumper().toggleOnTrue(m_autos.armIntakeCG(ArmPositions.FRONT_UP_FLOOR, false));

        m_driverController.rightTrigger().toggleOnTrue(m_autos.armIntakeCG(
                ArmPositions.BACK_TIPPED_FLOOR,
                ArmPositions.BACK_TIPPED_FLOOR_PRESTOW,
                false));

        m_driverController.leftBumper().toggleOnTrue(m_autos.armIntakeCG(
                ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT,
                ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT_PRESTOW,
                false));
        m_driverController.leftTrigger().onTrue(m_autos.armIntakeCG(ArmConstants.GROUND_CUBE_INTAKE_POSITION, true));

        m_driverController.x().whileTrue(m_drivebaseS.leftPlatformAlign());
        m_driverController.x().onTrue(m_autos.armIntakeCG(
            ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT,
            ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT_PRESTOW,
            false));
        m_driverController.y().onTrue(m_autos.armIntakeCG(
            ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT,
            ArmPositions.FRONT_PLATFORM_CONE_UPRIGHT_PRESTOW,
            false));
        m_driverController.y().whileTrue(m_drivebaseS.rightPlatformAlign());

        // Button on keypad for pulsing the intake in.
        m_keypad.action().onTrue(m_intakeS.intakeC(m_autos::isCubeSelected).withTimeout(0.5));
        m_keypad.enter().toggleOnTrue(m_autos.autoScoreSequenceCG());

        // D-pad driving slowly relative to intake direction.
        m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());
    }

    public void addAutoRoutines() {
        m_autoSelector.setDefaultOption("Do Nothing", none());
        m_autoSelector.addOption("3pc HP", m_autos.highConeHighCubeHPSide().andThen(m_autos.midCubeHPAddon()));
        m_autoSelector.addOption("3pc Bump", m_autos.highConeHighCubeBumpSide().andThen(m_autos.midCubeBumpAddon()));
        m_autoSelector.addOption("2.5pc Bump Climb",
                m_autos.highConeHighCubeBumpSide().andThen(m_autos.cubePickupClimbBumpAddon()));
        m_autoSelector.addOption("2cone.5cube Bump Climb", m_autos.highConeHighConeCubePickupClimb());

        m_autoSelector.addOption("BumpSide 1 Cone Bal", m_autos.highConeBalance(3));
        m_autoSelector.addOption("HP Side 1 Cone Bal", m_autos.highConeBalance(5));
        m_autoSelector.addOption("High Link Bump", m_autos.highConeHighCubeBumpSide().andThen(m_autos.highConeBumpAddon()));
    }

    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }

    @BothLog
    public boolean getSafeToPlace() {
        return m_autos.m_alignSafeToPlace.getAsBoolean();
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            enabledEntry.set(false);
            if (m_setupDone) {
                LightStripS.getInstance().requestState(States.SetupDone);
            } else {
                LightStripS.getInstance().requestState(States.Disabled);
            }
        } else {
            enabledEntry.set(true);
        }
        controller1Entry.set(DriverStation.isJoystickConnected(0));
        controller2Entry.set(DriverStation.isJoystickConnected(2));
        TimingTracer.update();
        loopTime = loopTimeAverage.calculate(TimingTracer.getLoopTime());
        //SmartDashboard.putNumber("loopTime", loopTime);

        LightStripS.getInstance()
                .requestState(m_autos.isCubeSelected() ? States.RequestingCube : States.RequestingCone);
        // /* Trace the loop duration and plot to shuffleboard */
        LightStripS.getInstance().periodic();
        updateFields();
        AutoLog.update();
    }

    public void updateFields() {
        m_drivebaseS.drawRobotOnField(m_field);
        m_driverField.setRobotPose(m_drivebaseS.getPose());
        m_driverField.getObject("target").setPose(m_autos.getTargetAlignmentPose());
        m_field.getObject("selection").setPose(m_autos.getTargetAlignmentPose());
        m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
    }

    public void onEnabled() {
        m_drivebaseS.resetRelativeRotationEncoders();
        m_armS.resetAllControllers();
    }

    public void onDisabled() {
    }

    public Command driveAlignPlat(Supplier<POIS> target) {
        return m_drivebaseS.run(() -> {
            double downfield = (AllianceWrapper.getAlliance() == Alliance.Red) ? Math.PI : 0.0;
            double rot = m_drivebaseS.m_thetaController.calculate(
                    m_drivebaseS.getPoseHeading().getRadians(), downfield);
            m_drivebaseS.driveFieldRelative(
                    new ChassisSpeeds(
                            m_fwdXAxis.getAsDouble() * Constants.DriveConstants.MAX_LINEAR_SPEED / 2
                                    * Math.cos(downfield),
                            MathUtil.clamp(
                                    m_drivebaseS.m_yController.calculate(
                                            m_drivebaseS.getPose().getY(),
                                            POIManager.ownPOI(target.get()).getY()
                                                    + 0.25 * m_rotAxis.getAsDouble() * Math.cos(downfield)),
                                    -1, 1),
                            rot));
        });
    }

    public Command driveIntakeRelativePOV() {
        return m_drivebaseS.run(() -> {
            double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
            if (m_armS.getArmPosition().pivotRadians > Math.PI) {
                pov += Math.PI;
            }
            double adjustSpeed = 0.75; // m/s
            m_drivebaseS.drive(
                    new ChassisSpeeds(
                            Math.cos(pov) * adjustSpeed,
                            Math.sin(pov) * adjustSpeed,
                            m_rotAxis.getAsDouble() * DriveConstants.MAX_TURN_SPEED));
        });
    }

}
