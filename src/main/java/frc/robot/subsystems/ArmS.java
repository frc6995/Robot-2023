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
import frc.robot.subsystems.arm.SimExtendIO;
import frc.robot.subsystems.arm.SimPivotIO;
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
    private double m_pivotAngle = 0;
    private double m_pivotVelocity = 0;
    private double m_wristAngle = 0;
    private double m_wristVelocity = 0;
    private double m_extendLength = 0;
    private double m_extendVelocity = 0;

    //public ArmConstraintsManager setpointManager = new ArmConstraintsManager(VISUALIZER);
    public ArmS(Consumer<Runnable> addPeriodic, Supplier<Double> handLengthSupplier) {
        this.handLengthSupplier = handLengthSupplier;
        if (RobotBase.isReal()) {
            m_extender = new RealExtendIO(addPeriodic);
            m_pivot = new RealPivotIO(addPeriodic);
        }
        else {
            m_extender = new SimExtendIO(addPeriodic);
            m_pivot = new SimPivotIO(addPeriodic);
        }
        m_extender.setAngleSupplier(m_pivot::getContinuousRangeAngle);
        m_pivot.setExtendLengthSupplier(m_extender::getLength);
        // Reset
        new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(m_extender::resetController).ignoringDisable(true));
        Command homingCommand = Commands.runOnce(()->{
            m_extender.onHome();
            m_extender.resetController();
        }).ignoringDisable(true);
        new Trigger(m_extender::isHomed).debounce(0.04).and(new Trigger(DriverStation::isDisabled))
            .onTrue(homingCommand);
        initWrist();
        initSimulation();
        initVisualizer();
        
        //SmartDashboard.putData(MECH_VISUALIZER);
        //SmartDashboard.putData(VISUALIZER);
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

    private void updateEncoders() {
        //m_pivotVelocity = m_pivotEncoderWrapper.getVelocity() * 2 * Math.PI / 60;
        m_wristAngle = MathUtil.angleModulus(m_wristEncoderWrapper.getPosition());
        //m_wristVelocity = m_wristEncoderWrapper.getVelocity();
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
        updateEncoders();  
        updateVisualizer();
    }

    /**
     * Finds the position of the hand relative to the robot in 3d space
     * @return returns position of the hand in 3d space
     */

    // public Transform3d getGamePieceTransform() {
    //     var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
    //     var wristPose = pivotPose.transformBy(
    //         new Transform2d(new Translation2d(getLengthMeters(), 0), getWristAngle())
    //     );
    //     var piecePose = wristPose.transformBy(new Transform2d(new Translation2d(HAND_LENGTH / 2, 0), new Rotation2d()));
    //     var transform3d = new Transform3d(
    //         new Translation3d(piecePose.getX(), 0, piecePose.getY()),
    //         new Rotation3d(0, -piecePose.getRotation().getRadians() ,0)
    //     );
    //     return transform3d;
    // }

    // region extend



    /**
     * Sets extender motor voltage to voltage parameter
     * @param voltage Desired motor voltage
     * @return run Command to set extender motor voltage
     */
    public Command extendC(DoubleSupplier voltage){
        return run(()->{m_extender.setVolts(voltage.getAsDouble());})
        .finallyDo((interrupted)->m_extender.setVolts(0));
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
        m_extender.resetController();
    }

    // endregion

    // region pivot

    public Command goToPositionC(ArmPosition position) {
        return new GoToPositionC(this, ()->position);
    }

    public Command goToPositionIndefiniteC(ArmPosition position) {
        return new GoToPositionC(this, ()->position, false);
    }

    public Command holdPositionC(){
        return new HoldCurrentPositionC(this);
    }

    // endregion

    // region wrist
    private final double wristMOI = HAND_MASS_KILOS * HAND_LENGTH * HAND_LENGTH / 3.0;
    private final CANSparkMax m_wristMotor = new CANSparkMax(WRIST_MOTOR_ID,MotorType.kBrushless);
    private final LinearSystem<N2, N1, N1> m_wristPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(1), wristMOI, 1.0/WRIST_ROTATIONS_PER_MOTOR_ROTATION);

    private final LinearPlantInversionFeedforward<N2, N1, N1> m_wristFeedforward
        = new LinearPlantInversionFeedforward<>(m_wristPlant, 0.02);
    
    private final SparkMaxAbsoluteEncoderWrapper m_wristEncoderWrapper = new SparkMaxAbsoluteEncoderWrapper(m_wristMotor, WRIST_ENCODER_OFFSET);
    private final ProfiledPIDController m_wristController = new ProfiledPIDController(
        2, 0, 0, new Constraints(4, 8));
    /**
     * initializes wrist:
     * sets position conversion factor for the wrist motor in rotations,</p>
     * sets velocity conversion factor for the wrist motor in rotations per second,</p>
     * sets soft limit for the maximum wrist angle, </p>
     * sets soft limit for the minimum wrist angle, </p>
     * sets encoder position to 0, </p>
     * resets the wrist profiled PID controller, </p>
     * sets the position and velocity tolerances for the profiled PID controller
     */

    public void initWrist() {
        m_wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(2 * Math.PI);
        m_wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setVelocityConversionFactor(2 * Math.PI/ 60);
        m_wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setInverted(true);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
        m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        // m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) WRIST_MAX_ANGLE);
        // m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) WRIST_MIN_ANGLE);
        m_wristMotor.setInverted(true);
        m_wristController.reset(getWristAngle().getRadians());
        m_wristController.setTolerance(0.05, 0.05);
        m_wristMotor.burnFlash();
    }

    /**
     * returns the wrist angle in radians
     * @return angle of the wrist as a Rotation2d
     */

    public Rotation2d getWristAngle() {
        return Rotation2d.fromRadians(getContinuousWristAngle());
    }

    /**
     * returns the wrist angle in radians
     * @return angle of the wrist in radians
     */

    @Log
    public double getContinuousWristAngle() {
        return m_wristAngle;
    }

    // /**
    //  * @return the current velocity of the wrist in rotations per minute
    //  */

    // @Log
    // public double getWristVelocity() {
    //     return m_wristVelocity;
    // }

    /**
     * sets the wrist velocity in radians per second to velocity parameter
     * @param velocityRadPerSec desired velocity in radians per second
     */

    public void setWristVelocity(double velocityRadPerSec) {
        setWristVolts(
            m_wristFeedforward.calculate(VecBuilder.fill(0, velocityRadPerSec)).get(0, 0)
            + getWristkGVolts()

        );
    }

    public void resetWrist() {
        double angle = getContinuousWristAngle();
        m_wristController.reset(angle);
        m_wristController.setGoal(angle);
    }

    /**
     * sets voltage of wrist motor to volts parameter
     * @param volts desired motor voltage
     */

    public void setWristVolts(double volts) {
        if (getContinuousWristAngle() > WRIST_MAX_ANGLE && volts > 0) {
            volts = 0;
        }
        if (getContinuousWristAngle() < WRIST_MIN_ANGLE && volts < 0) {
            volts = 0;
        }

        m_wristMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    /**
     * @return voltage required to counteract the force of gravity on the hand
     */

    public double getWristkGVolts() {
        // angle relative to world horizontal
        // = pivot angle + wrist angle
        return WRIST_KG * Math.cos(m_pivot.getContinuousRangeAngle() + getContinuousWristAngle());
    }

    /**
     * sets wrist angle in radiands to target angle parameter (straight out relative to arm is 0)
     * @param targetAngle desired angle in radians relative to arm
     */

    public void setWristAngle(double targetAngle) {
        targetAngle = MathUtil.angleModulus(targetAngle);
        setWristVelocity(
            m_wristController.calculate(
                getContinuousWristAngle(), targetAngle
            )
            + m_wristController.getSetpoint().velocity / 3
        );
    }

    /**
     * holds wrist at current angle relative to arm
     * @return run command
     */

    public Command holdWristC() {
        return run(()->setWristVolts(getWristkGVolts()));
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
    public Command stowC() {
        return new GoToPositionC(this, ()->STOW_POSITION);
    }
    public Command stowIndefiniteC() {
        return new GoToPositionC(this, ()->STOW_POSITION, false);
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
        });
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
        );
    }


    // endregion

    // region simulation

    private final VariableLengthArmSim m_wristSim = new VariableLengthArmSim(
        m_wristPlant,
        DCMotor.getNEO(1),
        1.0 / WRIST_ROTATIONS_PER_MOTOR_ROTATION,
        wristMOI, HAND_LENGTH,
        WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, HAND_MASS_KILOS, true);
    
    /**
     * initializes simulation:
     * sets velocity of pivot joint in simulation to 0, </p>
     * sets arm telescoping velocity in simulation to 0, </p>
     * sets wrist angle in simulation to 0, </p>
     * sets wrist velocity in simulation to 0, </p>
     * sets the angle that gravity acts on the arm in simulation to -90 degrees relative to the field
     */

    private void initSimulation() {;
        m_wristSim.setState(VecBuilder.fill(STOW_POSITION.wristRadians,0));
        //as the arm raises from 0 to pi/2, the gravity on the wrist goes from -pi/2 to -pi
        m_wristSim.setGravityAngle(-Math.PI/2 - m_pivot.getAngle().getRadians());
    }

     /**
     * runs every code loop in simulation, called in simulation periodic: </p>
     * updates the gravity angle of wrist in simulation, </p>
     * sets wrist input voltage in simulation to applied output, </p>
     * updates the wrist simulation by 0.02 seconds, </p>
     * updates encoder positions in simulations, </p>
     * updates encoder velocity in simulation
     */

    public void wristSimulationPeriodic() {
        m_wristSim.setGravityAngle(-Math.PI/2 - m_pivot.getAngle().getRadians());
        m_wristSim.setInputVoltage(MathUtil.clamp(DriverStation.isEnabled() ? m_wristMotor.getAppliedOutput() : 0, -12, 12));
        m_wristSim.update(TimingTracer.getLoopTime());
        m_wristEncoderWrapper.setSimPosition(m_wristSim.getAngleRads());
        m_wristEncoderWrapper.setSimVelocity(m_wristSim.getVelocityRadPerSec());

    }

    public void simulationPeriodic() {
        wristSimulationPeriodic();
        
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
        VISUALIZER.getObject("Current").setPose(new Pose2d(m_pivot.getContinuousRangeAngle(), m_extender.getLength(), getWristAngle()));
        VISUALIZER.getObject("setpoint").setPose(new Pose2d(m_pivot.getSetpoint().position, m_extender.getSetpoint().position, Rotation2d.fromRadians(m_wristController.getSetpoint().position)));
        VISUALIZER.getObject("goal").setPose(new Pose2d(m_pivot.getGoal().position, m_extender.getGoal().position, Rotation2d.fromRadians(m_wristController.getGoal().position)));


        MECH_VISUALIZER_ARM.setAngle(Units.radiansToDegrees(m_pivot.getContinuousRangeAngle()) - 90);
        MECH_VISUALIZER_ARM.setLength(m_extender.getLength());
        MECH_VISUALIZER_HAND.setAngle(getWristAngle());
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



