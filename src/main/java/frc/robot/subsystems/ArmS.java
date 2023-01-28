package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.ARM_EXTEND_KG_VERTICAL;
import static frc.robot.Constants.ArmConstants.ARM_MASS_KG;
import static frc.robot.Constants.ArmConstants.ARM_PIVOT_KG_MAX_EXTEND;
import static frc.robot.Constants.ArmConstants.ARM_PIVOT_KG_MIN_EXTEND;
import static frc.robot.Constants.ArmConstants.ARM_PIVOT_TRANSLATION;
import static frc.robot.Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION;
import static frc.robot.Constants.ArmConstants.EXTEND_DRUM_RADIUS;
import static frc.robot.Constants.ArmConstants.EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION;
import static frc.robot.Constants.ArmConstants.EXTEND_METERS_PER_DRUM_ROTATION;
import static frc.robot.Constants.ArmConstants.EXTEND_MOTOR_ID;
import static frc.robot.Constants.ArmConstants.HAND_LENGTH;
import static frc.robot.Constants.ArmConstants.HAND_MASS_KG;
import static frc.robot.Constants.ArmConstants.MAX_ARM_ANGLE;
import static frc.robot.Constants.ArmConstants.MAX_ARM_LENGTH;
import static frc.robot.Constants.ArmConstants.MIN_ARM_ANGLE;
import static frc.robot.Constants.ArmConstants.MIN_ARM_LENGTH;
import static frc.robot.Constants.ArmConstants.PIVOT_MOTOR_ID;
import static frc.robot.Constants.ArmConstants.WRIST_KG;
import static frc.robot.Constants.ArmConstants.WRIST_MAX_ANGLE;
import static frc.robot.Constants.ArmConstants.WRIST_MIN_ANGLE;
import static frc.robot.Constants.ArmConstants.WRIST_MOTOR_ID;
import static frc.robot.Constants.ArmConstants.WRIST_ROTATIONS_PER_MOTOR_ROTATION;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ArmS extends SubsystemBase implements Loggable {

    public ArmS() {
        initExtender();
        initPivot();
        initWrist();
        initSimulation();
        initVisualizer();
    }

    public void periodic() {      
        pivotPeriodic();
        updateVisualizer();
    }

    /**
     * Finds the position of the hand relative to the robot in 3d space
     * @return returns position of the hand in 3d space
     */

    public Transform3d getGamePieceTransform() {
        var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        var wristPose = pivotPose.transformBy(
            new Transform2d(new Translation2d(getLengthMeters(), 0), getWristAngle())
        );
        var piecePose = wristPose.transformBy(new Transform2d(new Translation2d(HAND_LENGTH / 2, 0), new Rotation2d()));
        var transform3d = new Transform3d(
            new Translation3d(piecePose.getX(), 0, piecePose.getY()),
            new Rotation3d(0, -piecePose.getRotation().getRadians() ,0)
        );
        return transform3d;
    }

    // region extend
    private final CANSparkMax m_extendMotor = new CANSparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_extendEncoderWrapper = new SparkMaxEncoderWrapper(m_extendMotor);
    private final LinearSystem<N2, N1, N1> m_extendPlant =
        LinearSystemId.identifyPositionSystem(
            12 / (Units.inchesToMeters(81.2)), 0.01);
    
    private final LinearPlantInversionFeedforward<N2,N1,N1> m_extendFeedforward
        = new LinearPlantInversionFeedforward<>(m_extendPlant, 0.02);

    private final ProfiledPIDController m_extendController =
        new ProfiledPIDController(5,0,0,
            new Constraints(2, 4)
        );

    /**
     * initializes extender: sets postion conversion factor of extender encoder, 
     * sets velocity conversion factor of extender encoder,
     * sets minimum and maximum soft limits for extender encoder,
     * sets initial position of encoder to minimum extension length,
     * sets profiled PID controller to minimum position
     */
    // TODO: add homing limitswitches

    public void initExtender() {
        m_extendMotor.getEncoder().setPositionConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION);
        m_extendMotor.getEncoder().setVelocityConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION / 60);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_LENGTH);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_LENGTH);
        m_extendEncoderWrapper.setPosition(MIN_ARM_LENGTH);
        m_extendController.reset(MIN_ARM_LENGTH);
    }

    public void extendPeriodic(){}

    /**
     * Sets voltage to the extend motor
     * @param volts voltage to apply to the extend motor
     */

    public void setExtendVolts(double volts) {
        m_extendMotor.setVoltage(volts);
    }

    /**
     * @return returns the distance from the pivot to the wrist joint in meters
     */
    @Log
    public double getLengthMeters() {
        return m_extendEncoderWrapper.getPosition();
    }

    /**
     * @return returns telescoping velocity of arm in meters per second
     */

    @Log
    public double getExtendVelocity(){
        return m_extendEncoderWrapper.getVelocity();
    }

    /**
     * sets telescoping velocity of arm using feedforward
     * @param velocityMetersPerSecond desired telescoping velocity in meters per second
     */

    public void setExtendVelocity(double velocityMetersPerSecond) {
        m_extendMotor.setVoltage(
            m_extendFeedforward.calculate(
                VecBuilder.fill(0, getExtendVelocity()),
                VecBuilder.fill(0, velocityMetersPerSecond)
            ).get(0,0)
            + ARM_EXTEND_KG_VERTICAL * Math.sin(getAngle().getRadians())
        );
    }

    /**
     * sets the desired arm length in meters
     * @param lengthMeters desired arm length in meters
     */

    public void setExtendLength(double lengthMeters) {
        setExtendVelocity(
            m_extendController.calculate(getLengthMeters(), lengthMeters)
            +m_extendController.getSetpoint().velocity
        );
    }

    /**
     * Sets extender motor voltage to voltage parameter
     * @param voltage Desired motor voltage
     * @return run Command to set extender motor voltage
     */
    public Command extendC(DoubleSupplier voltage){
        return run(()->{setExtendVolts(voltage.getAsDouble());})
        .finallyDo((interrupted)->setExtendVolts(0));
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

    // endregion

    // region pivot
    private final CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_pivotEncoderWrapper = new SparkMaxEncoderWrapper(m_pivotMotor);

    private LinearSystem<N2, N1, N1> m_pivotPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(2),  1.0 / 3.0 * ARM_MASS_KG * MIN_ARM_LENGTH * MIN_ARM_LENGTH
        , 1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION);

    private DCMotor m_pivotGearbox = DCMotor.getNEO(1);

    private LinearPlantInversionFeedforward<N2, N1, N1> m_pivotFeedForward
        = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);

    private ProfiledPIDController m_pivotController = new ProfiledPIDController(
        5, 0, 0, new Constraints(4,4));

    private double armStartAngle = 0;

    /**
     * Initialize pivot: 
     * Sets position conversion factor in rotations for pivot motor</p>
     * Sets velocity conversion factor in rotations/second for pivot motor</p>
     * Sets soft limits for minimum and maximum rotation</p>
     * Sets initial position of pivot encoder to armStartAngle</p>
     * Sets initial position of motor controller to armStartAngle</p>
     * Defines position and velocity tolerances</p>
     */
    private void initPivot() {
        m_pivotMotor.getEncoder().setPositionConversionFactor(ARM_ROTATIONS_PER_MOTOR_ROTATION);
        m_pivotMotor.getEncoder().setVelocityConversionFactor(ARM_ROTATIONS_PER_MOTOR_ROTATION / 60);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_ANGLE);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_ANGLE);

        m_pivotEncoderWrapper.setPosition(armStartAngle);

        m_pivotController.reset(armStartAngle);
        m_pivotController.setTolerance(0.05, 0.05);
    }

    /**
     * Periodically updates the pivot plant
     */

    private void pivotPeriodic() {
        updatePivotPlant();
        m_pivotFeedForward = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);
    }

    /**
     * Sets voltage of pivot motor to the volts parameter
     * @param volts Desired voltage
     */


    public void setPivotVolts(double volts) {
        m_pivotMotor.setVoltage(volts);
    }

    /**
     * @return Returns the angle of the pivot joint
     */

    @Log(methodName = "getRadians")
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_pivotEncoderWrapper.getPosition());
    }

    /**
     * @return Returns the velocity of the pivot joint
     */

    @Log
    public double getPivotVelocity() {
        return m_pivotEncoderWrapper.getVelocity();
    }

    /**
     * @return Returns the MOI of the arm in Joules per kg^2
     */

    public double getPivotMOI() {
        // TODO get this from held piece status and length
        return 1.0 / 3.0 * ARM_MASS_KG * getLengthMeters() * getLengthMeters();
    }

    public void updatePivotPlant() {
        m_pivotPlant.getA().set(1, 1, 
          -1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION * 1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION
          * m_pivotGearbox.KtNMPerAmp
          / (m_pivotGearbox.KvRadPerSecPerVolt * m_pivotGearbox.rOhms * getPivotMOI()));
        m_pivotPlant.getB().set(1, 0, 
          1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION * m_pivotGearbox.KtNMPerAmp / (m_pivotGearbox.rOhms * getPivotMOI()));
    }

    public void setPivotVelocity(double velocityRadPerSec) {
        setPivotVolts(m_pivotFeedForward.calculate(VecBuilder.fill(0, getPivotVelocity()), VecBuilder.fill(0, velocityRadPerSec)).get(0,0) + (getPivotkG() * getAngle().getCos()));
    }

    public void setPivotAngle(double targetAngle) {
        // We need to convert this to -90 to 270.
        // We don't want continuous input, but we need the rollover point to be outside our range of motion.
        targetAngle = MathUtil.angleModulus(targetAngle);
        // now in range -180 to 180
        if (targetAngle <= -Math.PI/2) {
            targetAngle += 2 * Math.PI;
        }
        SmartDashboard.putNumber("armRequestAngle", targetAngle);
        var outputVelocity = m_pivotController.calculate(
            getAngle().getRadians(),
            targetAngle
        );
        SmartDashboard.putNumber("armError", m_pivotController.getPositionError());
        SmartDashboard.putNumber("armRequestVel", outputVelocity + m_pivotController.getSetpoint().velocity);
        setPivotVelocity(outputVelocity + m_pivotController.getSetpoint().velocity);
    }

    @Log
    public double getPivotkG() {
        double minkG = ARM_PIVOT_KG_MIN_EXTEND;
        double maxkG = ARM_PIVOT_KG_MAX_EXTEND;

        double result = minkG;
        double frac = (getLengthMeters() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        result += frac * (maxkG - minkG);
        return result;
    }

    public Command holdC() {
        return run(()->setPivotAngle(0));
    }

    public Command counterClockwiseC() {
        return run(()->setPivotVelocity(0.5))
        .finallyDo((interrupted)->setPivotVolts(0));
    }

    public Command clockwiseC() {
        return run(()->setPivotVelocity(-0.5))
        .finallyDo((interrupted)->setPivotVolts(0));
    }
    // endregion

    // region wrist
    private final double wristMOI = HAND_MASS_KG * HAND_LENGTH * HAND_LENGTH / 3.0;
    private final CANSparkMax m_wristMotor = new CANSparkMax(WRIST_MOTOR_ID,MotorType.kBrushless);
    private final LinearSystem<N2, N1, N1> m_wristPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(1), wristMOI, 1.0/WRIST_ROTATIONS_PER_MOTOR_ROTATION);

    private final LinearPlantInversionFeedforward<N2, N1, N1> m_wristFeedforward
        = new LinearPlantInversionFeedforward<>(m_wristPlant, 0.02);
    
    private final SparkMaxEncoderWrapper m_wristEncoderWrapper = new SparkMaxEncoderWrapper(m_wristMotor);
    private final ProfiledPIDController m_wristController = new ProfiledPIDController(
        1, 0, 0, new Constraints(4, 4));

    public void initWrist() {
        m_wristMotor.getEncoder().setPositionConversionFactor(WRIST_ROTATIONS_PER_MOTOR_ROTATION);
        m_wristMotor.getEncoder().setVelocityConversionFactor(WRIST_ROTATIONS_PER_MOTOR_ROTATION / 60);
        m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) WRIST_MAX_ANGLE);
        m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) WRIST_MIN_ANGLE);

        m_wristEncoderWrapper.setPosition(0);

        m_wristController.reset(0);
        m_wristController.setTolerance(0.05, 0.05);
    }



    @Log(methodName="getRadians")
    public Rotation2d getWristAngle() {
        return Rotation2d.fromRadians(m_wristEncoderWrapper.getPosition());
    }

    public double getWristVelocity() {
        return m_wristEncoderWrapper.getVelocity();
    }

    public void setWristVelocity(double velocityRadPerSec) {
        m_wristMotor.setVoltage(
            m_wristFeedforward.calculate(VecBuilder.fill(0, velocityRadPerSec)).get(0, 0)
            + getWristkGVolts()

        );
    }

    public void setWristVolts(double volts) {
        m_wristMotor.setVoltage(volts);
    }

    public double getWristkGVolts() {
        // angle relative to world horizontal
        // = pivot angle + wrist angle
        return WRIST_KG * Math.cos(getAngle().plus(getWristAngle()).getRadians());
    }

    public void setWristAngle(double targetAngle) {
        targetAngle = MathUtil.angleModulus(targetAngle);
        setWristVelocity(
            m_wristController.calculate(
                getWristAngle().getRadians(), targetAngle
            ) + m_wristController.getSetpoint().velocity
        );
    }

    public Command holdWristC() {
        return run(()->setWristVolts(getWristkGVolts()));
    }
    // endregion

    // region factories
    public Command followTargetC() {
        return run(()->{
            var targetPose = VISUALIZER.getObject("2_target").getPose();
            var wristTargetPose = targetPose.transformBy(new Transform2d(
                new Translation2d(-HAND_LENGTH, 0),
                new Rotation2d()));
            var offset = wristTargetPose.getTranslation();
            offset = new Translation2d(offset.getX(), offset.getY() - Units.inchesToMeters(25));
            setPivotAngle(offset.getAngle().getRadians());
            setExtendLength(offset.getNorm());
            setWristAngle(-getAngle().getRadians() + targetPose.getRotation().getRadians());
        });
    }

    // endregion

    // region simulation

    private final VariableLengthArmSim m_pivotSim = new VariableLengthArmSim(
        m_pivotPlant,
        DCMotor.getNEO(1),
        1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION,
        1.0 / 3.0 * ARM_MASS_KG * MIN_ARM_LENGTH * MIN_ARM_LENGTH,
        MIN_ARM_LENGTH,
        MIN_ARM_ANGLE,
        MAX_ARM_ANGLE,
        ARM_MASS_KG,
        true
    );

    private final TiltedElevatorSim m_extendSim = new TiltedElevatorSim(
        m_extendPlant,
        DCMotor.getNEO(1),
        EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION,
        EXTEND_DRUM_RADIUS,
        MIN_ARM_LENGTH, MAX_ARM_LENGTH, true);

    private final VariableLengthArmSim m_wristSim = new VariableLengthArmSim(
        m_wristPlant,
        DCMotor.getNEO(1),
        1.0 / WRIST_ROTATIONS_PER_MOTOR_ROTATION,
        wristMOI, HAND_LENGTH,
        WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, HAND_MASS_KG, true);
    

    private void initSimulation() {
        m_pivotSim.setState(VecBuilder.fill(getAngle().getRadians(),0));
        m_extendSim.setState(VecBuilder.fill(getLengthMeters(), 0));
        m_wristSim.setState(VecBuilder.fill(0,0));
        //as the arm raises from 0 to pi/2, the gravity on the wrist goes from -pi/2 to -pi
        m_wristSim.setGravityAngle(-Math.PI/2 - getAngle().getRadians());
    }

    private void pivotSimulationPeriodic() {
        m_pivotSim.setCGRadius(getLengthMeters() / 2);
        m_pivotSim.setMOI(getPivotMOI());
        m_pivotSim.setInputVoltage(DriverStation.isEnabled() ? m_pivotMotor.getAppliedOutput() : 0);
        m_pivotSim.update(0.02);
        m_pivotEncoderWrapper.setSimPosition(m_pivotSim.getAngleRads());
        m_pivotEncoderWrapper.setSimVelocity(m_pivotSim.getVelocityRadPerSec());
    }

    public void extendSimulationPeriodic() {
        m_extendSim.setAngleFromHorizontal(getAngle().getRadians());
        m_extendSim.setInputVoltage(m_extendMotor.getAppliedOutput());
        m_extendSim.update(0.02);
        m_extendEncoderWrapper.setSimPosition(m_extendSim.getPositionMeters());
        m_extendEncoderWrapper.setSimVelocity(m_extendSim.getVelocityMetersPerSecond());
    }

    public void wristSimulationPeriodic() {
        m_wristSim.setGravityAngle(-Math.PI/2 - getAngle().getRadians());
        m_wristSim.setInputVoltage(DriverStation.isEnabled() ? m_wristMotor.getAppliedOutput() : 0);
        m_wristSim.update(0.02);
        m_wristEncoderWrapper.setSimPosition(m_wristSim.getAngleRads());
        m_wristEncoderWrapper.setSimVelocity(m_wristSim.getVelocityRadPerSec());

    }

    public void simulationPeriodic() {
        extendSimulationPeriodic();
        pivotSimulationPeriodic();
        wristSimulationPeriodic();
        
    }
    // endregion

    // region visualizer

    @Log
    public final Field2d VISUALIZER = new Field2d();

    public void initVisualizer() {
        var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        VISUALIZER.getObject("2_target").setPose(new Pose2d(
            pivotPose.getTranslation().plus(new Translation2d(0, MIN_ARM_LENGTH)),
            pivotPose.getRotation()
        ));
    }

    public void updateVisualizer() {
        var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        var wristPose = pivotPose.transformBy(
            new Transform2d(new Translation2d(getLengthMeters(), 0), new Rotation2d())
        );
        var handPose = wristPose.transformBy(new Transform2d(new Translation2d(), getWristAngle()));
        var handEndPose = handPose.transformBy(
            new Transform2d(new Translation2d(HAND_LENGTH, 0), new Rotation2d())
        );
        VISUALIZER.getObject("0_Pivot").setPose(pivotPose);
        VISUALIZER.getObject("1_Arm").setPoses(List.of(pivotPose, wristPose));
        VISUALIZER.getObject("2_Hand").setPoses(List.of(handPose, handEndPose));
    }

    // endregion


}

