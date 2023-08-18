package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.Robot;
import frc.robot.commands.arm.GoToPositionC;
import frc.robot.commands.arm.HoldCurrentPositionC;
import frc.robot.subsystems.arm.ExtendIO;
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
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ArmS extends SubsystemBase implements Loggable {
    
    public final Field2d VISUALIZER = new Field2d();
    private Supplier<Double> handLengthSupplier;

    private ExtendIO m_extender;
    private PivotIO m_pivot;
    private WristIO m_wrist;

    public ArmS(Consumer<Runnable> addPeriodic, Supplier<Double> handLengthSupplier) {
        this.handLengthSupplier = handLengthSupplier;
        if (RobotBase.isReal()) {
            m_extender = new RealExtendIO(addPeriodic);
            m_pivot = new RealPivotIO(addPeriodic);
            m_wrist = new RealWristIO(addPeriodic);
        }
        else {
            m_extender = new SimExtendIO(addPeriodic);
            m_pivot = new SimPivotIO(addPeriodic);
            m_wrist = new SimWristIO(addPeriodic);
        }
        m_extender.setAngleSupplier(m_pivot::getContinuousRangeAngle);
        m_wrist.setPivotAngleSupplier(m_pivot::getContinuousRangeAngle);
        m_pivot.setExtendLengthSupplier(m_extender::getLength);
        // Homing setup for extension
        new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(m_extender::resetController).ignoringDisable(true));
        Command homingCommand = Commands.runOnce(()->{
            m_extender.onHome();
            m_extender.resetController();
        }).ignoringDisable(true);
        new Trigger(m_extender::isHomed).debounce(0.04).and(new Trigger(DriverStation::isDisabled))
            .onTrue(homingCommand);
        
        initVisualizer();
        
        setDefaultCommand(new HoldCurrentPositionC(this));
        VISUALIZER.getObject("positions").setPoses(
            SCORE_HIGH_CONE_POSITION.asPose(),
            SCORE_MID_CONE_POSITION.asPose(),
            STOW_POSITION.asPose(),
            GROUND_CUBE_INTAKE_POSITION.asPose(),
            GROUND_CONE_INTAKE_POSITION.asPose(),
            RAMP_CONE_INTAKE_POSITION.asPose(),
            RAMP_CUBE_INTAKE_POSITION.asPose(),
            PLATFORM_CONE_INTAKE_POSITION.asPose(),
            OVERTOP_CONE_INTAKE_POSITION.asPose()


        );
        //setDefaultCommand(followJointSpaceTargetC());
    }

    public double constrainLength(double length) {
        return constrainLength(length, m_pivot.getContinuousRangeAngle());
    }
    public double constrainLength(double length, double angle) {
        return Math.max(getMinLength(angle), length);
    }

    public double getMinLength(double angle) {
        if (angle < 0 || angle > Math.PI) {
            return 0.610;
        }
        return MIN_ARM_LENGTH + Units.inchesToMeters(0.125);
    }

    public void periodic() {
        updateVisualizer();
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

    // region pivot

    public Command goToPositionC(ArmPosition position) {
        return goToPositionC(()->position);
    }

    public Command goToPositionC(Supplier<ArmPosition> position) {
        return new GoToPositionC(this, position).asProxy();
    }

    public Command goToPositionIndefiniteC(ArmPosition position) {
        return goToPositionIndefiniteC(()->position);
    }

    public Command goToPositionIndefiniteC(Supplier<ArmPosition> position) {
        return new GoToPositionC(this, position, false).asProxy();
    }

    public Command holdPositionC(){
        return new HoldCurrentPositionC(this).asProxy();
    }

    // endregion

    // region wrist
    /**
     * returns the wrist angle in radians
     * @return angle of the wrist in radians
     */

    @Log
    public double getContinuousWristAngle() {
        return m_wrist.getAngle();
    }

    // /**
    //  * @return the current velocity of the wrist in rotations per minute
    //  */

    // @Log
    // public double getWristVelocity() {
    //     return m_wristVelocity;
    // }

    public void resetWrist() {
        m_wrist.resetController();
    }

    /**
     * sets wrist angle in radiands to target angle parameter (straight out relative to arm is 0)
     * @param targetAngle desired angle in radians relative to arm
     */

    public void setWristAngle(double targetAngle) {
        m_wrist.setAngle(targetAngle);
    }

    /**
     * holds wrist at current angle relative to arm
     * @return run command
     */

    public Command holdWristC() {
        return run(m_wrist::openLoopHold).asProxy();
    }
    // endregion

    public ArmPosition getArmPosition() {
        return new ArmPosition(m_pivot.getContinuousRangeAngle(), m_extender.getLength(), getContinuousWristAngle(), HAND_LENGTH);
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
        }).asProxy();
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
    public Command stowC() {
        return goToPositionC(()->STOW_POSITION);
    }
    public Command stowIndefiniteC() {
        return goToPositionIndefiniteC(()->STOW_POSITION);
    }

    public Command followJointSpaceTargetC() {
        return run(()->{
            var currentTranslation = new Translation2d(m_pivot.getAngle().getRadians(), m_extender.getLength());
            var targetPose = VISUALIZER.getObject("target").getPose();
            var target = targetPose.getTranslation();
            
            var targetPoint = target; //pathTranslations.get(1);
            double armAngle = targetPoint.getX();
            boolean flipArm = armAngle > Math.PI/2;
            double wristAngle = targetPose.getRotation().minus(m_pivot.getAngle()).getRadians();
            m_pivot.setAngle(armAngle);
            m_extender.setLength(targetPoint.getY());
            setWristAngle(wristAngle);
        }).asProxy();
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
        Shuffleboard.getTab("ArmS").add("MECH_VISUALIZER", MECH_VISUALIZER);
        Shuffleboard.getTab("ArmS").add("VISUALIZER", VISUALIZER);
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
        MECH_VISUALIZER_HAND.setLength(handLengthSupplier.get());
    }

    
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
        public final double handLength;
        public ArmPosition(double pivotRadians, double armLength, double wristRadians, double handLength) {
            this.pivotRadians = pivotRadians;
            this.armLength = armLength;
            this.wristRadians = wristRadians;
            this.handLength = handLength;
        }

        public Pose2d asPose() {
            return new Pose2d(this.pivotRadians, this.armLength, Rotation2d.fromRadians(this.wristRadians));
        }

    }

}
