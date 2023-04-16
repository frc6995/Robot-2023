package frc.robot.subsystems;

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
import static frc.robot.Constants.ArmConstants.ARM_EXTEND_KG_VERTICAL;
import static frc.robot.Constants.ArmConstants.ARM_EXTEND_KS;
import static frc.robot.Constants.ArmConstants.ARM_EXTEND_KV;
import static frc.robot.Constants.ArmConstants.ARM_MASS_KILOS;
import static frc.robot.Constants.ArmConstants.ARM_MOI_SHRUNK;
import static frc.robot.Constants.ArmConstants.ARM_PIVOT_KG_MAX_EXTEND;
import static frc.robot.Constants.ArmConstants.ARM_PIVOT_KG_MIN_EXTEND;
import static frc.robot.Constants.ArmConstants.ARM_PIVOT_TRANSLATION;
import static frc.robot.Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION;
import static frc.robot.Constants.ArmConstants.EXTEND_DRUM_RADIUS;
import static frc.robot.Constants.ArmConstants.EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION;
import static frc.robot.Constants.ArmConstants.EXTEND_METERS_PER_DRUM_ROTATION;
import static frc.robot.Constants.ArmConstants.EXTEND_MOTOR_ID;
import static frc.robot.Constants.ArmConstants.GROUND_CONE_INTAKE_POSITION;
import static frc.robot.Constants.ArmConstants.GROUND_CUBE_INTAKE_POSITION;
import static frc.robot.Constants.ArmConstants.HAND_LENGTH;
import static frc.robot.Constants.ArmConstants.HAND_MASS_KILOS;
import static frc.robot.Constants.ArmConstants.MAX_ARM_ANGLE;
import static frc.robot.Constants.ArmConstants.MAX_ARM_LENGTH;
import static frc.robot.Constants.ArmConstants.MIN_ARM_ANGLE;
import static frc.robot.Constants.ArmConstants.MIN_ARM_LENGTH;
import static frc.robot.Constants.ArmConstants.OVERTOP_CONE_INTAKE_POSITION;
import static frc.robot.Constants.ArmConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ArmConstants.PIVOT_FOLLOWER_MOTOR_ID;
import static frc.robot.Constants.ArmConstants.PIVOT_KS;
import static frc.robot.Constants.ArmConstants.PIVOT_MAX_ACCEL;
import static frc.robot.Constants.ArmConstants.PIVOT_MAX_VELOCITY;
import static frc.robot.Constants.ArmConstants.PIVOT_MOTOR_ID;
import static frc.robot.Constants.ArmConstants.PLATFORM_CONE_INTAKE_POSITION;
import static frc.robot.Constants.ArmConstants.RAMP_CONE_INTAKE_POSITION;
import static frc.robot.Constants.ArmConstants.RAMP_CUBE_INTAKE_POSITION;
import static frc.robot.Constants.ArmConstants.SCORE_HIGH_CONE_POSITION;
import static frc.robot.Constants.ArmConstants.SCORE_MID_CONE_POSITION;
import static frc.robot.Constants.ArmConstants.STOW_POSITION;
import static frc.robot.Constants.ArmConstants.WRIST_ENCODER_OFFSET;
import static frc.robot.Constants.ArmConstants.WRIST_KG;
import static frc.robot.Constants.ArmConstants.WRIST_MAX_ANGLE;
import static frc.robot.Constants.ArmConstants.WRIST_MIN_ANGLE;
import static frc.robot.Constants.ArmConstants.WRIST_MOTOR_ID;
import static frc.robot.Constants.ArmConstants.WRIST_ROTATIONS_PER_MOTOR_ROTATION;
import frc.robot.Robot;
import frc.robot.commands.arm.GoToPositionC;
import frc.robot.commands.arm.HoldCurrentPositionC;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;
import frc.robot.util.sim.SparkMaxAbsoluteEncoderWrapper;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ArmS extends SubsystemBase implements Loggable {
    
    public final Field2d VISUALIZER = new Field2d();
    private Supplier<Double> handLengthSupplier;

    private double m_pivotAngle = 0;
    private double m_pivotVelocity = 0;
    private double m_wristAngle = 0;
    private double m_wristVelocity = 0;
    private double m_extendLength = 0;
    private double m_extendVelocity = 0;
    //public ArmConstraintsManager setpointManager = new ArmConstraintsManager(VISUALIZER);
    public ArmS(Supplier<Double> handLengthSupplier) {
        this.handLengthSupplier = handLengthSupplier;
        initExtender();
        initPivot();
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
        m_extendLength = Math.max(MIN_ARM_LENGTH-0.25, m_extendEncoderWrapper.getPosition());
        m_extendVelocity = m_extendEncoderWrapper.getVelocity();
        m_pivotAngle = continuousRangeAngleModulus((m_pivotEncoderWrapper.getPosition() * 2 * Math.PI) - PIVOT_ENCODER_OFFSET);
        //m_pivotVelocity = m_pivotEncoderWrapper.getVelocity() * 2 * Math.PI / 60;
        m_wristAngle = MathUtil.angleModulus(m_wristEncoderWrapper.getPosition());
        //m_wristVelocity = m_wristEncoderWrapper.getVelocity();
    }

    public double constrainLength(double length) {
        return constrainLength(length, getContinuousRangeAngle());
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
        pivotPeriodic();
        //Translation2d[] path = setpointManager.calculatePath(new Translation2d(getAngle().getRadians(), getLengthMeters()),
        //VISUALIZER.getObject("pathTarget").getPose().getTranslation();
        // var poseList = new ArrayList<Pose2d> ();
        // for (int i = 0; i < path.length; i++) {
        //     poseList.add(new Pose2d(path[i], new Rotation2d()));
        // }
        // VISUALIZER.getObject("path").setPoses(poseList);
        //constraints.update(Units.inc);
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
    private final CANSparkMax m_extendMotor = new CANSparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_extendEncoderWrapper = new SparkMaxEncoderWrapper(m_extendMotor);
    //@Log(methodName="isPressed")
    private final SparkMaxLimitSwitch m_extendHomingSwitch = m_extendMotor.getReverseLimitSwitch(Type.kNormallyClosed);
    private final LinearSystem<N2, N1, N1> m_extendPlant =
        LinearSystemId.identifyPositionSystem(
            ARM_EXTEND_KV,
            0.01);
    
    private final SimpleMotorFeedforward m_extendFeedforward
        = new SimpleMotorFeedforward(ARM_EXTEND_KS, ARM_EXTEND_KV, 0.01);

    private final ProfiledPIDController m_extendController =
        new ProfiledPIDController(8,0,0,
            new Constraints(1.3, 3),
            0.02
        );

    /**
     * initializes extender: sets postion conversion factor of extender encoder, 
     * sets velocity conversion factor of extender encoder,
     * sets minimum and maximum soft limits for extender encoder,
     * sets initial position of encoder to minimum extension length,
     * sets profiled PID controller to minimum position
     */

    public void initExtender() {
        m_extendMotor.restoreFactoryDefaults();
        m_extendMotor.getEncoder().setPositionConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION);
        m_extendMotor.getEncoder().setVelocityConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION / 60);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_LENGTH);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_LENGTH);
        m_extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_extendHomingSwitch.enableLimitSwitch(true);
        m_extendMotor.setIdleMode(IdleMode.kCoast);
        m_extendMotor.setSmartCurrentLimit(40);
        m_extendMotor.burnFlash();
        m_extendEncoderWrapper.setPosition(MIN_ARM_LENGTH);
        m_extendController.reset(MIN_ARM_LENGTH);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        m_extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        
        Command homingCommand = new InstantCommand(()->{
            m_extendEncoderWrapper.setPosition(MIN_ARM_LENGTH);
            m_extendController.reset(MIN_ARM_LENGTH);
            m_extendMotor.setIdleMode(IdleMode.kBrake);
            // set state to current length (just reset above) and 0 velocity
            m_extendSim.setState(VecBuilder.fill(getLengthMeters(), 0));
        }).ignoringDisable(true);
        new Trigger(m_extendHomingSwitch::isPressed).debounce(0.04).and(new Trigger(DriverStation::isDisabled))
        .onTrue(homingCommand);
        m_extendMotor.burnFlash();
    }

    public void extendPeriodic(){}

    public void resetExtender() {
        m_extendController.reset(getLengthMeters());
    }

    /**
     * Sets voltage to the extend motor
     * @param volts voltage to apply to the extend motor
     */

    public void setExtendVolts(double volts) {
        //SmartDashboard.putNumber("armExtendVolts", volts);
        m_extendMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }


    @Log
    public double getExtendVolts(){
        return m_extendMotor.getAppliedOutput() * 12;
    }
    /**
     * @return returns the distance from the pivot to the wrist joint in meters
     */
    @Log
    public double getLengthMeters() {
        return m_extendLength;
    }

    @Log
    public double getExtendCurrent(){
        return m_extendMotor.getOutputCurrent();
    }

    /**
     * @return returns telescoping velocity of arm in meters per second
     */

    @Log
    public double getExtendVelocity(){
        return m_extendVelocity;
    }

    /**
     * sets telescoping velocity of arm using feedforward
     * @param velocityMetersPerSecond desired telescoping velocity in meters per second
     */

    public void setExtendVelocity(double velocityMetersPerSecond) {
        setExtendVolts(
            m_extendFeedforward.calculate(
                getExtendVelocity(),
                velocityMetersPerSecond, TimingTracer.getLoopTime()
            )
            + ARM_EXTEND_KG_VERTICAL * Math.sin(getContinuousRangeAngle())
            //+ ARM_EXTEND_KS * Math.signum(velocityMetersPerSecond)
        );
        //SmartDashboard.putNumber("extendRequestVelocity", velocityMetersPerSecond);
    }

    /**
     * sets the desired arm length in meters
     * @param lengthMeters desired arm length in meters
     */

    public void setExtendLength(double lengthMeters) {
        
        setExtendVelocity(
            m_extendController.calculate(getLengthMeters(), lengthMeters)
            
            + m_extendController.getSetpoint().velocity
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
    private final CANSparkMax m_pivotFollowerMotor = new CANSparkMax(PIVOT_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    //@Log(methodName="getPosition")
    private final SparkMaxAbsoluteEncoderWrapper m_pivotEncoderWrapper = new SparkMaxAbsoluteEncoderWrapper(m_pivotMotor, 0);

    private LinearSystem<N2, N1, N1> m_pivotPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(2),  ARM_MOI_SHRUNK
        , 1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION);

    private DCMotor m_pivotGearbox = DCMotor.getNEO(1);

    private LinearPlantInversionFeedforward<N2, N1, N1> m_pivotFeedForward
        = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);

    private ProfiledPIDController m_pivotController = new ProfiledPIDController(
        4, 0, 0.00, new Constraints(PIVOT_MAX_VELOCITY,PIVOT_MAX_ACCEL));

    private double armStartAngle = PIVOT_ENCODER_OFFSET;

    /**
     * Initialize pivot: 
     * Sets position conversion factor in rotations for pivot motor</p>
     * Sets velocity conversion factor in rotations/second for pivot motor</p>
     * Sets soft limits for minimum and maximum rotation</p>
     * Sets initial position of pivot encoder to armStartAngle</p>
     * Sets initial position of profile PID controller to armStartAngle</p>
     * Defines position and velocity tolerances of profiled PID controller</p>
     */
    private void initPivot() {
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotFollowerMotor.restoreFactoryDefaults();
        m_pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(1);
        m_pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setVelocityConversionFactor(1);
        // m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_ANGLE);
        // m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_ANGLE);
        m_pivotMotor.setInverted(true);
        m_pivotFollowerMotor.follow(m_pivotMotor, true);

        m_pivotEncoderWrapper.setSimPosition(PIVOT_ENCODER_OFFSET / (2 * Math.PI));
        CommandScheduler.getInstance().schedule(Commands.waitSeconds(1).andThen(
            Commands.runOnce(()->{
                m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 25);
                m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
                m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
                m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
                m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
                m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
                m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
                m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 45);
                m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
                m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        
                m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
                m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
                m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
                m_pivotFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        
                m_pivotController.reset(getContinuousRangeAngle());
                m_pivotController.setTolerance(0.05, 0.05);
                m_pivotMotor.burnFlash();
            })).ignoringDisable(true)
            
        );

    }

    /**
     * Periodically updates the pivot plant
     */

    private void pivotPeriodic() {
        if(Robot.isSimulation()){
            updatePivotPlant();
        }

        //m_pivotFeedForward = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);
        //m_pivotController.setConstraints(new Constraints(PIVOT_MAX_VELOCITY, getMaxPivotAcceleration()));
       
    }
    // public double getMaxPivotAcceleration() {
    //     double t = (getLengthMeters() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
    //     return MathUtil.interpolate(PIVOT_MAX_ACCEL_RETRACTED, PIVOT_MAX_ACCEL_EXTENDED, t);
    // }

    @Log
    public double getPivotVolts(){
        return m_pivotMotor.getAppliedOutput() * 12;
    }
    /**
     * Sets voltage of pivot motor to the volts parameter
     * @param volts Desired voltage
     */


    public void setPivotVolts(double volts) {

        if (getContinuousRangeAngle() < MIN_ARM_ANGLE && volts < 0) {
            volts = 0;
        }
        if (getContinuousRangeAngle() > MAX_ARM_ANGLE && volts > 0) {
            volts = 0;
        }
        //volts = MathUtil.clamp(volts, -2, 2);
        m_pivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    /**
     * @return the angle of the pivot joint
     */

    //@Log(methodName = "getRadians")
    public Rotation2d getAngle() {
        double position = getContinuousRangeAngle();
        return Rotation2d.fromRadians(position);
    }

    @Log
    public double getContinuousRangeAngle() {
        return m_pivotAngle;
    }

    public double continuousRangeAngleModulus(double targetAngle) {
        targetAngle = MathUtil.angleModulus(targetAngle);
        // now in range -180 to 180
        if (targetAngle <= -Math.PI/2) {
            targetAngle += 2 * Math.PI;
        }
        return targetAngle;
    }

    // /**
    //  * @return the velocity of the pivot joint
    //  */
    // @Log
    // public double getPivotVelocity() {
    //     return m_pivotVelocity;
    // }

    /**
     * @return the MOI of the arm in Joules per kg^2
     */

    public double getPivotMOI() {
        var moiAboutCM = 1.0/12.0 * ARM_MASS_KILOS * getLengthMeters() * getLengthMeters();
        return moiAboutCM;
        // TODO get this from held piece status and length
        // return (1.0 / 3.0 * ARM_MASS_KILOS * (getLengthMeters()-Units.inchesToMeters(12.5)) * (getLengthMeters()-Units.inchesToMeters(12.5)) / 2.0) + 
        // (1.0 / 3.0 * ARM_MASS_KILOS * Units.inchesToMeters(12.5) * Units.inchesToMeters(12.5) / 2.0); 
    }

    public double getPivotCGRadius() {
        double minCG = 0;
        double maxCG = Units.inchesToMeters(8);

        double result = minCG;
        double frac = (getLengthMeters() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        result += frac * (maxCG - minCG);
        return result;
    }

    public void resetPivot() {
        double angle =getContinuousRangeAngle();
        m_pivotController.reset(angle);
        m_pivotController.setGoal(angle);
    }
    /**
     * updates the pivot plant
     */

    public void updatePivotPlant() {
        m_pivotPlant.getA().set(1, 1, 
          -1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION * 1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION
          * m_pivotGearbox.KtNMPerAmp
          / (m_pivotGearbox.KvRadPerSecPerVolt * m_pivotGearbox.rOhms * getPivotMOI()));
        m_pivotPlant.getB().set(1, 0, 
          1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION * m_pivotGearbox.KtNMPerAmp / (m_pivotGearbox.rOhms * getPivotMOI()));
    }

    /**
     * sets pivot motor to desired velocity in radians per second
     * @param velocityRadPerSec desired velocity in radians per second
     */

    public void setPivotVelocity(double velocityRadPerSec) {
        var currentVoltageAdd = (getPivotkG() * getAngle().getCos())
        + PIVOT_KS * Math.signum(velocityRadPerSec);
        setPivotVolts(m_pivotFeedForward.calculate(
            VecBuilder.fill(0, velocityRadPerSec))
            .get(0,0)
            + currentVoltageAdd);
        //SmartDashboard.putNumber("armCommandVelocity", velocityRadPerSec);
    }

    /**
     * sets pivot joint to desired angle in radians
     * @param targetAngle desired angle in radians
     */

    public void setPivotAngle(double targetAngle) {
        // We need to convert this to -90 to 270.
        // We don't want continuous input, but we need the rollover point to be outside our range of motion.
        
        targetAngle = continuousRangeAngleModulus(targetAngle);
        // SmartDashboard.putNumber("armRequestAngle", targetAngle);
        var outputVelocity = m_pivotController.calculate(
            getContinuousRangeAngle(),
            targetAngle
        );
        // SmartDashboard.putNumber("armError", m_pivotController.getPositionError());
        // SmartDashboard.putNumber("armRequestVel", outputVelocity + m_pivotController.getSetpoint().velocity);
        setPivotVelocity(outputVelocity + m_pivotController.getSetpoint().velocity);
    }

    /**
     * calculates voltage required to counteract the force of gravity on the arm 
     * by interpolating between minimum and maximum arm lengths
     * @return returns the required voltage needed
     * to hold arm horizontal at current extension length
     */

    public double getPivotkG() {
        double minkG = ARM_PIVOT_KG_MIN_EXTEND;
        double maxkG = ARM_PIVOT_KG_MAX_EXTEND;
        double result = minkG;
        double frac = (getLengthMeters() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        result += frac * (maxkG - minkG);
        return result;
    }

    @Log
    public double getPivotCurrent() {
        return m_pivotMotor.getOutputCurrent();
    }

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
        return WRIST_KG * Math.cos(getContinuousRangeAngle() + getContinuousWristAngle());
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
        return new ArmPosition(getContinuousRangeAngle(), getLengthMeters(), getContinuousWristAngle(), HAND_LENGTH);
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
            setPivotAngle(offset.getAngle().getRadians());
            setExtendLength(offset.getNorm());
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
            var currentTranslation = new Translation2d(getAngle().getRadians(), getLengthMeters());
            var targetPose = VISUALIZER.getObject("target").getPose();
            var target = targetPose.getTranslation();
            
            var targetPoint = target; //pathTranslations.get(1);
            double armAngle = targetPoint.getX();
            boolean flipArm = armAngle > Math.PI/2;
            double wristAngle = targetPose.getRotation().minus(getAngle()).getRadians();
            setPivotAngle(armAngle);
            setExtendLength(targetPoint.getY());
            setWristAngle(wristAngle);
        });
    }

    public Command followJointSpaceTargetC(Supplier<ArmPosition> positionSupplier) {
        return Commands.sequence(
        runOnce(()->{
            resetExtender();
            resetPivot();
            resetWrist();
        }),
        run(()->{
            var position = positionSupplier.get();
            setPivotAngle(position.pivotRadians);
            setExtendLength(position.armLength);
            setWristAngle(position.wristRadians);
        }
        )
        );
    }


    // endregion

    // region simulation

    private final VariableLengthArmSim m_pivotSim = new VariableLengthArmSim(
        m_pivotPlant,
        DCMotor.getNEO(1),
        1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION,
        1.0 / 3.0 * ARM_MASS_KILOS * MIN_ARM_LENGTH * MIN_ARM_LENGTH,
        MIN_ARM_LENGTH,
        MIN_ARM_ANGLE,
        MAX_ARM_ANGLE,
        ARM_MASS_KILOS,
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
        WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, HAND_MASS_KILOS, true);
    
    /**
     * initializes simulation:
     * sets velocity of pivot joint in simulation to 0, </p>
     * sets arm telescoping velocity in simulation to 0, </p>
     * sets wrist angle in simulation to 0, </p>
     * sets wrist velocity in simulation to 0, </p>
     * sets the angle that gravity acts on the arm in simulation to -90 degrees relative to the field
     */

    private void initSimulation() {
        m_pivotSim.setState(VecBuilder.fill(STOW_POSITION.pivotRadians,0));
        m_extendSim.setState(VecBuilder.fill(STOW_POSITION.armLength, 0));
        m_wristSim.setState(VecBuilder.fill(STOW_POSITION.wristRadians,0));
        //as the arm raises from 0 to pi/2, the gravity on the wrist goes from -pi/2 to -pi
        m_wristSim.setGravityAngle(-Math.PI/2 - getAngle().getRadians());
    }

    /**
     * runs every code loop in simulation, called in simulation periodic: </p>
     * sets distance between pivot joint and the arm's center of gravity
     *  to half the length of the arm in simulation, </p>
     * updates the pivot's moment of inertia in simulation, </p>
     * sets pivot input voltage in simulation to applied output, </p>
     * updates the pivot simulation by 0.02 seconds, </p>
     * updates encoder positions in simulations, </p>
     * updates encoder velocity in simulation
     */

    private void pivotSimulationPeriodic() {
        m_pivotSim.setCGRadius(getPivotCGRadius());
        m_pivotSim.setMOI(getPivotMOI());
        double volts = MathUtil.clamp(DriverStation.isEnabled() ? m_pivotMotor.getAppliedOutput() : 0, -12, 12);
        m_pivotSim.setInputVoltage(NomadMathUtil.subtractkS(volts, PIVOT_KS));
        m_pivotSim.update(TimingTracer.getLoopTime());
        m_pivotEncoderWrapper.setSimPosition(Units.radiansToRotations(m_pivotSim.getAngleRads() + PIVOT_ENCODER_OFFSET));
        m_pivotEncoderWrapper.setSimVelocity(Units.radiansPerSecondToRotationsPerMinute(m_pivotSim.getVelocityRadPerSec()));
    }

     /**
     * runs every code loop in simulation, called in simulation periodic: </p>
     * updates the gravity angle of arm in simulation, </p>
     * sets extender input voltage in simulation to applied output, </p>
     * updates the extender simulation by 0.02 seconds, </p>
     * updates encoder positions in simulations, </p>
     * updates encoder velocity in simulation
     */

    public void extendSimulationPeriodic() {
        m_extendSim.setAngleFromHorizontal(getAngle().getRadians());
        m_extendSim.setInputVoltage(NomadMathUtil.subtractkS(MathUtil.clamp(m_extendMotor.getAppliedOutput(), -12, 12), 
            ARM_EXTEND_KS + (ARM_EXTEND_KG_VERTICAL * Math.sin(getContinuousRangeAngle()))));
        m_extendSim.update(TimingTracer.getLoopTime());
        m_extendEncoderWrapper.setSimPosition(m_extendSim.getPositionMeters());
        m_extendEncoderWrapper.setSimVelocity(m_extendSim.getVelocityMetersPerSecond());
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
        m_wristSim.setGravityAngle(-Math.PI/2 - getAngle().getRadians());
        m_wristSim.setInputVoltage(MathUtil.clamp(DriverStation.isEnabled() ? m_wristMotor.getAppliedOutput() : 0, -12, 12));
        m_wristSim.update(TimingTracer.getLoopTime());
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

    /**
     * initializes visualizer: </p>
     * sets up visualization field 2d
     */

    public void initVisualizer() {
        //var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        // VISUALIZER.getObject("2_target").setPose(new Pose2d(
        //     pivotPose.getTranslation().plus(new Translation2d(0, MIN_ARM_LENGTH)),
        //     pivotPose.getRotation()
        // ));
        // VISUALIZER.getObject("pathTarget")
        // .setPose(new Pose2d(getAngle().getRadians(), getLengthMeters(), new Rotation2d()));
        // VISUALIZER.getObject("target")
        // .setPose(new Pose2d(getAngle().getRadians(), getLengthMeters(), new Rotation2d()));
        // VISUALIZER.getObject("LOS")
        // .setPoses(new Pose2d(), new Pose2d(0, 1, new Rotation2d()));
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
        // var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        // var wristPose = pivotPose.transformBy(
        //     new Transform2d(new Translation2d(getLengthMeters(), 0), new Rotation2d())
        // );
        // var handPose = wristPose.transformBy(new Transform2d(new Translation2d(), getWristAngle()));
        // var handEndPose = handPose.transformBy(
        //     new Transform2d(new Translation2d(HAND_LENGTH, 0), new Rotation2d())
        // );
        VISUALIZER.getObject("Current").setPose(new Pose2d(getContinuousRangeAngle(), getLengthMeters(), getWristAngle()));
        // Pose2d[] losTester = new Pose2d[2];
        // losTester = VISUALIZER.getObject("LOS").getPoses().toArray(losTester);
        // if (losTester.length >= 2) {
        //     SmartDashboard.putBoolean("lineofSight", ArmConstraintsManager.belowConstraints(
        //         losTester[0].getTranslation(),
        //         losTester[1].getTranslation()));
        //     List<Translation2d> pathTranslations = ArmConstraintsManager.solvePath(
        //         losTester[0].getTranslation(),
        //         losTester[1].getTranslation()
        //         );
        //     List<Pose2d> pathPoses = new ArrayList<>();
        //     pathTranslations.forEach((Translation2d translation) -> {
        //         pathPoses.add(new Pose2d(translation, new Rotation2d()));
        //     });
        //     VISUALIZER.getObject("pathCandidates").setPoses(pathPoses);
        // }
        // VISUALIZER.getObject("0_Pivot").setPose(pivotPose);
        // VISUALIZER.getObject("1_Arm").setPoses(List.of(pivotPose, wristPose));
        // VISUALIZER.getObject("2_Hand").setPoses(List.of(handPose, handEndPose));
        VISUALIZER.getObject("setpoint").setPose(new Pose2d(m_pivotController.getSetpoint().position, m_extendController.getSetpoint().position, Rotation2d.fromRadians(m_wristController.getSetpoint().position)));
        VISUALIZER.getObject("goal").setPose(new Pose2d(m_pivotController.getGoal().position, m_extendController.getGoal().position, Rotation2d.fromRadians(m_wristController.getGoal().position)));


        MECH_VISUALIZER_ARM.setAngle(Units.radiansToDegrees(getContinuousRangeAngle()) - 90);
        MECH_VISUALIZER_ARM.setLength(getLengthMeters());
        MECH_VISUALIZER_HAND.setAngle(getWristAngle());
        MECH_VISUALIZER_HAND.setLength(handLengthSupplier.get());
    }

    
    private final Mechanism2d MECH_VISUALIZER = new Mechanism2d(Units.feetToMeters(12), Units.feetToMeters(8));
    private final MechanismRoot2d MECH_VISUALIZER_ROOT = MECH_VISUALIZER.getRoot("root", Units.feetToMeters(6), 0);
    private final MechanismLigament2d MECH_VISUALIZER_PIVOT_BASE = new MechanismLigament2d(
        "base", ARM_PIVOT_TRANSLATION.getNorm(), ARM_PIVOT_TRANSLATION.getAngle().getDegrees(),
        4, new Color8Bit(255, 255,255));
    private final MechanismLigament2d MECH_VISUALIZER_ARM = new MechanismLigament2d(
        "arm", MIN_ARM_LENGTH, armStartAngle);
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



