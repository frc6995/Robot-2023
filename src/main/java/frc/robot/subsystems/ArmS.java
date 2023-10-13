package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import autolog.Logged;
import autolog.AutoLog.BothLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.Robot;
import frc.robot.commands.arm.GoToPositionC;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.arm.ExtendIO;
import frc.robot.subsystems.arm.JITBWristIO;
import frc.robot.subsystems.arm.OffboardExtendIO;
import frc.robot.subsystems.arm.PivotIO;
import frc.robot.subsystems.arm.RealExtendIO;
import frc.robot.subsystems.arm.RealPivotIO;
import frc.robot.subsystems.arm.RealWristIO;
import frc.robot.subsystems.arm.SimExtendIO;
import frc.robot.subsystems.arm.SimPivotIO;
import frc.robot.subsystems.arm.SimWristIO;
import frc.robot.subsystems.arm.WristIO;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;
import frc.robot.util.sim.SparkMaxAbsoluteEncoderWrapper;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import autolog.Logged;
import io.github.oblarg.oblog.annotations.Log;

public class ArmS extends SubsystemBase implements Logged {
    
    public final Field2d VISUALIZER = new Field2d();

    private ExtendIO m_extender;
    private PivotIO m_pivot;
    private WristIO m_wrist;
    @BothLog
    private double[] m_position = new double[] {0, MIN_ARM_LENGTH, 0};
    private DigitalInput m_coastModeButton = new DigitalInput(0);
    private Trigger m_coastModeTrigger = (
        new CommandXboxController(0).start().or(
            ()->!m_coastModeButton.get()
        )
    ).and(DriverStation::isDisabled);

    public ArmS(Consumer<Runnable> addPeriodic, BooleanSupplier hasCone) {
        if (RobotBase.isReal()) {
            Timer.delay(0.1);
            m_extender = new OffboardExtendIO(addPeriodic);
            Timer.delay(0.1);
            m_pivot = new RealPivotIO(addPeriodic, hasCone);
            Timer.delay(0.2);
            m_wrist = new JITBWristIO(addPeriodic, hasCone);
        }
        else {
            m_extender = new SimExtendIO(addPeriodic);
            m_pivot = new SimPivotIO(addPeriodic, hasCone);
            m_wrist = new SimWristIO(addPeriodic, hasCone);
        }
        m_extender.setAngleSupplier(m_pivot::getContinuousRangeAngle);
        m_wrist.setPivotAngleSupplier(m_pivot::getContinuousRangeAngle);
        m_pivot.setExtendLengthSupplier(m_extender::getLength);
        m_coastModeTrigger.whileTrue(coastC());
        // Homing setup for extension
        new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(m_extender::resetController).ignoringDisable(true));
        Command homingCommand = Commands.runOnce(()->{
            m_extender.onHome();
            m_extender.resetController();
        }).ignoringDisable(true);
        new Trigger(m_extender::isHomed).and(m_coastModeTrigger.negate()).debounce(0.04).and(new Trigger(DriverStation::isDisabled))
            .onTrue(homingCommand);
        m_position = getPosition();
        initVisualizer();
        
        //setDefaultCommand(new HoldCurrentPositionC(this));
        VISUALIZER.getObject("positions").setPoses(
            SCORE_HIGH_CONE_POSITION.asPose(),
            SCORE_MID_CONE_POSITION.asPose(),
            STOW_POSITION.asPose(),
            GROUND_CUBE_INTAKE_POSITION.asPose()
        );
        //setDefaultCommand(followJointSpaceTargetC());
    }

    public void periodic() {
        updatePosition();
        updateVisualizer();
    }

    public void resetAllControllers() {
        resetExtender();
        resetPivot();
        resetWrist();
        m_extender.resetGoal();
        m_pivot.resetGoal();
        m_wrist.resetGoal();
    }

    /**
     * Sets extender motor voltage to voltage parameter
     * @param voltage Desired motor voltage
     * @return run Command to set extender motor voltage
     */
    public Command extendC(DoubleSupplier voltage){
        return run(()->{m_extender.setVolts(voltage.getAsDouble());})
        .finallyDo((interrupted)->m_extender.setVolts(0)).asProxy();
    }

    public void setExtendLength(double lengthMeters) {
        m_extender.setLength(lengthMeters);
    }
    public void setAngle(double lengthMeters) {
        m_pivot.setAngle(lengthMeters);
    }
    /**
     * Sets forward voltage to 3 to extender motor
     * @return run Command to set forward voltage to 3 to extender motor
     */
    public Command extendC() {
        return extendC(()->3);
    }

    /**
     * Sets reverse voltage to -3 to extender motor
     * @return run Command to set reverse voltage to -3 to extender motor
     */
    public Command retractC() {
        return extendC(()->-3);
    }

    public void resetExtender() {
        m_extender.resetController();
    }
    public void resetPivot() {
        m_pivot.resetController();
    }

    // endregion

    public Command goToPositionC(ArmPosition position) {
        return goToPositionC(()->position);
    }

    public Command goToPositionC(Supplier<ArmPosition> position) {
        return new GoToPositionC(this, position);
    }

    public Command goToPositionIndefiniteC(ArmPosition position) {
        return goToPositionIndefiniteC(()->position);
    }

    public Command goToPositionIndefiniteC(Supplier<ArmPosition> position) {
        return new GoToPositionC(this, position, false);
    }

    /**
     * returns the wrist angle in radians
     * @return angle of the wrist in radians
     */

    @BothLog
    public double getContinuousWristAngle() {
        return m_wrist.getAngle();
    }

    public void resetWrist() {
        m_wrist.resetController();
    }

    public void markWristHomed() {
        m_wrist.endHome();
    }

    /**
     * sets wrist angle in radians to target angle parameter (straight out relative to arm is 0)
     * @param targetAngle desired angle in radians relative to arm
     */

    public void setWristAngle(double targetAngle) {
        m_wrist.setAngle(targetAngle);
    }

    public double[] getPosition() {
        return m_position;
    }

    private void updatePosition() {
        m_position[0] = getContinuousWristAngle();
        m_position[1] = m_extender.getLength();
        m_position[2] = getContinuousWristAngle();
    }

    // endregion

    public ArmPosition getArmPosition() {
        return new ArmPosition(m_pivot.getContinuousRangeAngle(), m_extender.getLength(), getContinuousWristAngle());
    }
    // region factories

    /**
     * follow target position in vertical plane
     * @param poseSupplier the Pose2d in the robot's XZ plane for the end of the hand to be at
     * @return run command
     */

    public Command followTargetC(Supplier<Pose2d> poseSupplier) {
        return run(()->{
            var targetPose = poseSupplier.get();
            var wristTargetPose = targetPose.transformBy(new Transform2d(
                new Translation2d(-HAND_LENGTH, 0),
                new Rotation2d()));
            var offset = wristTargetPose.getTranslation();
            offset = new Translation2d(offset.getX(), offset.getY() - ARM_PIVOT_TRANSLATION.getY());
            m_pivot.setAngle(offset.getAngle().getRadians());
            m_extender.setLength(offset.getNorm());
            setWristAngle(-offset.getAngle().getRadians());
        });
    }

    // public Command scoreHighConeC() {
    //     return followTargetC(()->new Pose2d(Units.inchesToMeters(48 + 12.5), Units.inchesToMeters(56), new Rotation2d()));
    // }
    // public Command scoreMidConeC() {
    //     return followTargetC(()->new Pose2d(Units.inchesToMeters(31 + 12.5), Units.inchesToMeters(44), new Rotation2d()));
    // }

    // public Command scoreHighCubeC() {
    //     return followJointSpaceTargetC(()->SCORE_HIGH_CONE_POSITION);
    // }

    public Command coastC() {
        return runOnce(()->{
            m_extender.setIdleMode(IdleMode.kCoast);
            m_wrist.setIdleMode(IdleMode.kCoast);
            m_pivot.setIdleMode(IdleMode.kCoast);
        }).andThen(
            LightStripS.getInstance().stateC(()->States.ArmAdjust)
        ).finallyDo((interrupted)->{
            m_extender.setIdleMode(IdleMode.kBrake);
            m_wrist.setIdleMode(IdleMode.kBrake);
            m_pivot.setIdleMode(IdleMode.kBrake);
        }).ignoringDisable(true);
    }
    public Command stowC() {
        return goToPositionC(()->ArmPositions.STOW);
    }
    public Command stowIndefiniteC() {
        return new GoToPositionC(this, ()->ArmPositions.STOW, false);
    }

    public Command followJointSpaceTargetC(Supplier<ArmPosition> positionSupplier) {
        return Commands.sequence(
        runOnce(()->{
            m_extender.resetController();
            m_pivot.resetPivot();
            resetWrist();
        }),
        run(()->{
            var position = positionSupplier.get();
            m_pivot.setAngle(position.pivotRadians);
            m_extender.setLength(position.armLength);
            setWristAngle(position.wristRadians);
        }
        )
        ).asProxy();
    }


    // endregion

    // region visualizer

    /**
     * initializes visualizer: </p>
     * sets up visualization field 2d
     */

    public void initVisualizer() {
        //NetworkTableInstance.getDefault().
        //Shuffleboard.getTab("ArmS").add("MECH_VISUALIZER", MECH_VISUALIZER);
        //Shuffleboard.getTab("ArmS").add("VISUALIZER", VISUALIZER);
        initMechVisualizer();
    }

    /**
     * updates visualizer: </p>
     * updates position of pivot joint in visualizer, </p>
     * updates position of arm in visualizer, </p>
     * updates the position of the hand in visualizer, </p>
     */

    public void updateVisualizer() {
        VISUALIZER.getObject("Current").setPose(new Pose2d(
            m_pivot.getContinuousRangeAngle(),
            m_extender.getLength(),
            new Rotation2d(m_wrist.getAngle())
        ));
        VISUALIZER.getObject("setpoint").setPose(new Pose2d(
            m_pivot.getSetpoint().position,
            m_extender.getSetpoint().position,
            Rotation2d.fromRadians(m_wrist.getSetpoint().position)));
        VISUALIZER.getObject("goal").setPose(new Pose2d(
            m_pivot.getGoal().position,
            m_extender.getGoal().position,
            Rotation2d.fromRadians(m_wrist.getGoal().position)));


        MECH_VISUALIZER_ARM.setAngle(Units.radiansToDegrees(m_pivot.getContinuousRangeAngle()) - 90);
        MECH_VISUALIZER_ARM.setLength(m_extender.getLength());
        MECH_VISUALIZER_HAND.setAngle(Units.radiansToDegrees(m_wrist.getAngle()));
    }

    @BothLog
    private final Mechanism2d MECH_VISUALIZER = new Mechanism2d(Units.feetToMeters(12), Units.feetToMeters(8));
    private final MechanismRoot2d MECH_VISUALIZER_ROOT = MECH_VISUALIZER.getRoot("root", Units.feetToMeters(6), 0);
    private final MechanismLigament2d MECH_VISUALIZER_PIVOT_BASE = new MechanismLigament2d(
        "base", ARM_PIVOT_TRANSLATION.getNorm(), ARM_PIVOT_TRANSLATION.getAngle().getDegrees(),
        4, new Color8Bit(255, 255,255));
    private final MechanismLigament2d MECH_VISUALIZER_ARM = new MechanismLigament2d(
        "arm", MIN_ARM_LENGTH, 0);
    private final MechanismLigament2d MECH_VISUALIZER_HAND = new MechanismLigament2d(
            "hand", HAND_LENGTH, 0);
    
    private void initMechVisualizer() {
        MECH_VISUALIZER_ROOT
        .append(MECH_VISUALIZER_PIVOT_BASE)
        .append(MECH_VISUALIZER_ARM)
        .append(MECH_VISUALIZER_HAND)
        ;
    }
    
    // endregion

    
    public static class ArmPosition {
        public final double pivotRadians;
        public final double armLength;
        public final double wristRadians;
        public ArmPosition(double pivotRadians, double armLength, double wristRadians) {
            this.pivotRadians = pivotRadians;
            this.armLength = armLength;
            this.wristRadians = wristRadians;
        }

        public Pose2d asPose() {
            return new Pose2d(this.pivotRadians, this.armLength, Rotation2d.fromRadians(this.wristRadians));
        }

    }

}
