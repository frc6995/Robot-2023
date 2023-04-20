package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.AZMTH_ENC_COUNTS_PER_MODULE_REV;
import static frc.robot.Constants.DriveConstants.AZMTH_REVS_PER_ENC_REV;
import static frc.robot.Constants.DriveConstants.BL;
import static frc.robot.Constants.DriveConstants.BR;
import static frc.robot.Constants.DriveConstants.FL;
import static frc.robot.Constants.DriveConstants.FR;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.POIManager.POIS;
import frc.robot.subsystems.LightS.States;
import frc.robot.subsystems.VisionWrapper.VisionMeasurement;
import frc.robot.Constants;
import frc.robot.POIManager;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PoseEstimator;

import static frc.robot.Constants.DriveConstants.NUM_MODULES;
import static frc.robot.Constants.DriveConstants.ROBOT_MASS_kg;
import static frc.robot.Constants.DriveConstants.ROBOT_MOI_KGM2;
import static frc.robot.Constants.DriveConstants.WHEEL_BASE_WIDTH_M;
import static frc.robot.Constants.DriveConstants.WHEEL_ENC_COUNTS_PER_WHEEL_REV;
import static frc.robot.Constants.DriveConstants.WHEEL_RADIUS_M;
import static frc.robot.Constants.DriveConstants.WHEEL_REVS_PER_ENC_REV;
import frc.robot.Constants.VisionConstants;
import frc.robot.NavX.AHRS;
import frc.robot.POIManager.POIS;
import frc.robot.Robot;
import frc.robot.subsystems.LightS.States;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.sim.SimGyroSensorModel;
import frc.robot.util.sim.wpiClasses.QuadSwerveSim;
import frc.robot.util.sim.wpiClasses.SwerveModuleSim;
import frc.robot.util.trajectory.PPChasePoseCommand;
import frc.robot.util.trajectory.PPHolonomicDriveController;
import frc.robot.util.trajectory.PPSwerveControllerCommand;
import frc.robot.vision.PhotonCameraWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


/**
 * Subsystem that controls the drivetrain of the robot
 * Handles all the odometry and base movement for the chassis
 */
public class DrivebaseS extends SubsystemBase implements Loggable {
    private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 50);
    private SimGyroSensorModel m_simNavx = new SimGyroSensorModel();

    public final PIDController m_xController = new PIDController(3, 0, 0);
    public final PIDController m_yController = new PIDController(3, 0, 0);
    public final PIDController m_thetaController = new PIDController(3, 0, 0);
    // constraints determined from OperatorControlC slew settings.
    public final ProfiledPIDController m_profiledThetaController = 
        new ProfiledPIDController(3, 0, 0, new Constraints(2*Math.PI, 4*Math.PI));
    public final PPHolonomicDriveController m_holonomicDriveController = new PPHolonomicDriveController(m_xController, m_yController, m_thetaController);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        ModuleConstants.FL.centerOffset,
        ModuleConstants.FR.centerOffset,
        ModuleConstants.BL.centerOffset,
        ModuleConstants.BR.centerOffset
    );

    /**
     * odometry for the robot, measured in meters for linear motion and radians for rotational motion
     * Takes in kinematics and robot angle for parameters
     */
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDriveOdometry m_odometry;
    private final PhotonCameraWrapper m_camera1Wrapper;
    private final PhotonCameraWrapper m_camera2Wrapper;

    private final VisionWrapper m_visionWrapper = new VisionWrapper();

    private final List<SwerveModuleSim> m_moduleSims = List.of(
        DrivebaseS.swerveSimModuleFactory(),
        DrivebaseS.swerveSimModuleFactory(),
        DrivebaseS.swerveSimModuleFactory(),
        DrivebaseS.swerveSimModuleFactory()
    );

    private final QuadSwerveSim m_quadSwerveSim = 
    new QuadSwerveSim(
        WHEEL_BASE_WIDTH_M,
        WHEEL_BASE_WIDTH_M,
        ROBOT_MASS_kg,
        ROBOT_MOI_KGM2,
        m_moduleSims
    );
    private SwerveModule m_fl = new SwerveModule(ModuleConstants.FL);
    private SwerveModule m_fr = new SwerveModule(ModuleConstants.FR);
    private SwerveModule m_bl = new SwerveModule(ModuleConstants.BL);
    private SwerveModule m_br = new SwerveModule(ModuleConstants.BR);
    @Log.Exclude
    private final List<SwerveModule> m_modules = List.of(
        m_fl, m_fr, m_bl, m_br
    );

    private SwerveModuleState[] currentStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private SwerveModulePosition[] currentPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private Pose2d multitagPose = new Pose2d();

    public DrivebaseS() {
        m_navx.reset();
        m_navx.enableLogging(true);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), currentPositions);
        m_poseEstimator =
        new SwerveDrivePoseEstimator(
            m_kinematics, getHeading(), getModulePositions(), new Pose2d(),
            Constants.PoseEstimator.STATE_STANDARD_DEVIATIONS,
            Constants.PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS);
        m_thetaController.setTolerance(Units.degreesToRadians(0.5));
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_profiledThetaController.setTolerance(Units.degreesToRadians(0.5));
        m_profiledThetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_xController.setTolerance(0.01);
        m_yController.setTolerance(0.01);
        m_camera1Wrapper = new PhotonCameraWrapper(VisionConstants.CAM_1_NAME, VisionConstants.robotToCam1);
        m_camera2Wrapper = new PhotonCameraWrapper(VisionConstants.CAM_2_NAME, VisionConstants.robotToCam2);
        
        //resetPose(POIManager.mirrorPose(new Pose2d(1.835, 1.072, Rotation2d.fromRadians(Math.PI))));
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(new Quaternion(
            m_navx.getQuaternionW(),
            m_navx.getQuaternionX(),
            m_navx.getQuaternionY(),
            m_navx.getQuaternionZ()
        ));
    }

    @Log
    public double getGyroHeading() {
        return m_navx.getAngle();
    }

    @Log
    public double getPitch() {
        return Units.degreesToRadians(m_navx.getPitch());
    }

    @Override
    public void periodic() {
        updateModuleStates();
        updateModulePositions();

        // var cam1Pose = m_camera1Wrapper.getEstimatedGlobalPose(getPose());
        // if (cam1Pose.getFirst() != null) {
        //     var pose = cam1Pose.getFirst();
        //     var timestamp = cam1Pose.getSecond();
        //     m_poseEstimator.addVisionMeasurement(pose, timestamp);
        // }

        // var cam2Pose = m_camera2Wrapper.getEstimatedGlobalPose(getPose());
        // if (cam2Pose.getFirst() != null) {
        //     var pose = cam2Pose.getFirst();
        //     var timestamp = cam2Pose.getSecond();
        //     m_poseEstimator.addVisionMeasurement(pose, timestamp);
        // }



        // update the odometry every 20ms
        //m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.9, 0.9, 0.));
        m_odometry.update(getHeading(), currentPositions);
        m_poseEstimator.update(getHeading(), getModulePositions());

        VisionMeasurement measurement;
        while ((measurement = m_visionWrapper.drainVisionMeasurement()) != null) {
            var estimation = measurement.estimation();
            var estimatedPose = estimation.estimatedPose;
            // height of final pose
            if (Math.abs(estimatedPose.getZ()) > 0.5) {
                continue;
            }

            if (estimation.targetsUsed.size() < 2 && estimation.targetsUsed.get(0).getPoseAmbiguity() > 0.2) {
                continue;
            }
            // if (estimation.targetsUsed.size() < 2 && estimation.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() > Units.feetToMeters(12)) {
            //     continue;
            // }

            if (estimation.targetsUsed.size() < 2 && Math.abs(estimatedPose.toPose2d().getRotation().minus(getPoseHeading()).getRadians()) > Units.degreesToRadians(5)) {
                continue;
            }
    
            multitagPose = measurement.estimation().estimatedPose.toPose2d();
            m_poseEstimator.addVisionMeasurement(
                multitagPose,
                measurement.estimation().timestampSeconds,
                measurement.confidence());
        }
    }

    public void drive(ChassisSpeeds speeds) {
        // use kinematics (wheel placements) to convert overall robot state to array of individual module states
        SwerveModuleState[] states;

        // If we are stopped (no wheel velocity commanded) then any number of wheel angles could be valid.
        // By default it would point all modules forward when stopped. Here, we override this.
        if(Math.abs(speeds.vxMetersPerSecond) < 0.01
            && Math.abs(speeds.vyMetersPerSecond) < 0.01
            && Math.abs(speeds.omegaRadiansPerSecond) < 0.0001 && false) {
                states = getStoppedStates();
        } else {
            // make sure the wheels don't try to spin faster than the maximum speed possible
            states = m_kinematics.toSwerveModuleStates(speeds);
            // SwerveDriveKinematics.desaturateWheelSpeeds(states, speeds,
            //     MAX_FWD_REV_SPEED_MPS,
            //     MAX_ROTATE_SPEED_RAD_PER_SEC,
            //     MAX_MODULE_SPEED_FPS);
        } 
        setModuleStates(states);
    }


    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPoseHeading()));
    }

    public void driveAllianceRelative(ChassisSpeeds fieldRelativeSpeeds) {
        if (AllianceWrapper.getAlliance() == Alliance.Red) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPoseHeading().plus(Rotation2d.fromRadians(Math.PI))));
        }
        else {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPoseHeading()));
        }
    }

    public void driveFieldRelativeHeading(ChassisSpeeds speeds) {
        double omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
        double currentTargetRadians = m_thetaController.getSetpoint();
        
        double newTargetRadians = currentTargetRadians + (omegaRadiansPerSecond/50);

        double commandRadiansPerSecond = 
        m_thetaController.calculate(getPoseHeading().getRadians(), newTargetRadians);

        speeds.omegaRadiansPerSecond = commandRadiansPerSecond;
        driveFieldRelative(speeds);
    }


    /**
     * method for driving the robot
     * Parameters:
     * forward linear value
     * sideways linear value
     * rotation value
     * if the control is field relative or robot relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {       

        /**
         * ChassisSpeeds object to represent the overall state of the robot
         * ChassisSpeeds takes a forward and sideways linear value and a rotational value
         * 
         * speeds is set to field relative or default (robot relative) based on parameter
         */
        ChassisSpeeds speeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, getPoseHeading())
                : new ChassisSpeeds(forward, strafe, rotation);
        
        drive(speeds);
        
    }

    /**
     * Return the desired states of the modules when the robot is stopped. This can be an x-shape to hold against defense,
     * or all modules forward. Here we have it stopping all modules but leaving the angles at their current positions.
     * 
     * 
     * @return
     */
    private SwerveModuleState[] getStoppedStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = m_modules.get(i).getCurrentState();
            states[i].speedMetersPerSecond = 0;
        }
        return states;
    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < NUM_MODULES; i++) {
            m_modules.get(i).setDesiredStateClosedLoop(moduleStates[i]);
        }
    }

    private void updateModuleStates() {
        for (int i = 0; i < NUM_MODULES; i++) {
            currentStates[i] = m_modules.get(i).getCurrentState();
        }
    }

    private void updateModulePositions() {
        for (int i = 0; i < NUM_MODULES; i++) {
            currentPositions[i] = m_modules.get(i).getCurrentPosition();
        }
    }
    /*
     *  returns an array of SwerveModuleStates. 
     *  Front(left, right), Rear(left, right)
     *  This order is important to remain consistent across the codebase, or commands can get swapped around.
     */
    public SwerveModuleState[] getModuleStates() {
        

        return currentStates;
    }

    /**
     * Return the module positions for odometry.
     * @return an array of 4 SwerveModulePosition objects
     */
    public SwerveModulePosition[] getModulePositions() {

        return currentPositions;
    }

    /**
     * Return the current position of the robot on field
     * Based on drive encoder and gyro reading
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Return the simulated estimate of the robot's pose.
     * NOTE: on a real robot this will return a new Pose2d, (0, 0, 0)
     * @return
     */
    public Pose2d getSimPose() {
        if(Robot.isSimulation()) {
            return m_quadSwerveSim.getCurPose();
        }
        else {
            return new Pose2d();
        }
    }

    public Command resetPoseToBeginningC(PathPlannerTrajectory trajectory) {
        return Commands.runOnce(()->resetPose(NomadMathUtil.mirrorPose(
            new Pose2d(
                trajectory.getInitialState().poseMeters.getTranslation(),
                trajectory.getInitialState().holonomicRotation
            ), AllianceWrapper.getAlliance()
            )
        ));
    }

    /**
     * Reset the pose of odometry and sim to the given pose.
     * @param pose The Pose2d to reset to.
     */
    public void resetPose(Pose2d pose) {
        m_quadSwerveSim.modelReset(pose);
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose );
        m_odometry.resetPosition(getHeading(), currentPositions, pose);
    }

    // reset the measured distance driven for each module
    public void resetDriveDistances() {
        m_modules.forEach((module)->module.resetDistance());
    }

    /**
     * @return the current navX heading (which will not match odometry after drift or reset)
     */
    public Rotation2d getHeading() {
        if(Robot.isSimulation()) {
            return m_simNavx.getRotation2d();
        }
        return m_navx.getRotation2d();
    }

    @Log
    public double getHeadingDouble() {
        return getHeading().getRadians();
    }

    /**
     * Gets the current heading based on odometry. (this value will reflect odometry resets)
     * @return the current odometry heading.
     */
    public Rotation2d getPoseHeading() {
        return getPose().getRotation();
    }
    
    /*
     * Resets the navX to 0 position;
     */
    public void resetImu() {
        m_navx.reset();
        if (Robot.isSimulation()) {
            m_simNavx.resetToPose(new Pose2d());
        }

    }
 
    public void setRotationState(double radians) {
        m_thetaController.setSetpoint(radians);
    }

    /** Returns a Translation2d representing the linear robot speed in field coordinates. */
    public Translation2d getFieldRelativeLinearSpeedsMPS() {
        // Get robot relative speeds from module states
        ChassisSpeeds robotRelativeSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
        // Get field relative speeds by undoing the field-robot conversion (which was just a rotation by the heading)
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            getPoseHeading().unaryMinus()
        );
        // Convert to translation
        Translation2d translation = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        // to avoid angle issues near 0, if the distance is 0.01 or less just return (0, 0)
        if (translation.getNorm() < 0.01) {
            return new Translation2d();
        }
        else {
            return translation;
        }
    }

    @Override
    public void simulationPeriodic() {
        
        // set inputs. Set 0 if the robot is disabled.
        if(!DriverStation.isEnabled()){
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                m_moduleSims.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                double azmthVolts = m_modules.get(idx).getAppliedRotationVoltage();
                double wheelVolts = NomadMathUtil.subtractkS(m_modules.get(idx).getAppliedDriveVoltage(), 0) * 1.44;
                m_moduleSims.get(idx).setInputVoltages(wheelVolts, azmthVolts);
            }
        }

        Pose2d prevRobotPose = m_quadSwerveSim.getCurPose();

        // Update model (several small steps)
        for (int i = 0; i< 40; i++) {
            m_quadSwerveSim.update(0.0005);
        }
        

        //Set the state of the sim'd hardware
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            double azmthPos = m_moduleSims.get(idx).getAzimuthEncoderPositionRev();
            azmthPos = azmthPos / AZMTH_ENC_COUNTS_PER_MODULE_REV * 2 * Math.PI;
            double wheelPos = m_moduleSims.get(idx).getWheelEncoderPositionRev();
            wheelPos = wheelPos / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI * WHEEL_RADIUS_M;

            double wheelVel = m_moduleSims.get(idx).getWheelEncoderVelocityRevPerSec();
            wheelVel = wheelVel / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI * WHEEL_RADIUS_M;
            m_modules.get(idx).setSimState(azmthPos, wheelPos, wheelVel);
           
        }
        // Set the gyro based on the difference between the previous pose and this pose.
        m_simNavx.update(m_quadSwerveSim.getCurPose(), prevRobotPose);
    }

    /**
     * A convenience method to draw the robot pose and 4 poses representing the wheels onto the field2d.
     * @param field
     */
    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        field.getObject("odometry").setPose(getOdometryPose());
        field.getObject("multitag").setPose(multitagPose);
        // Draw a pose that is based on the robot pose, but shifted by the translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        field.getObject("modules").setPoses(List.of(
            getPose().transformBy(new Transform2d(ModuleConstants.FL.centerOffset, getModuleStates()[FL].angle)),
            getPose().transformBy(new Transform2d(ModuleConstants.FR.centerOffset ,getModuleStates()[FR].angle)),
            getPose().transformBy(new Transform2d(ModuleConstants.BL.centerOffset, getModuleStates()[BL].angle)),
            getPose().transformBy(new Transform2d(ModuleConstants.BR.centerOffset, getModuleStates()[BR].angle))
        ));

    }

    static SwerveModuleSim swerveSimModuleFactory(){
        return new SwerveModuleSim(DCMotor.getNEO(1), 
                                   DCMotor.getNEO(1), 
                                   WHEEL_RADIUS_M,
                                   1.0/AZMTH_REVS_PER_ENC_REV, // steering motor rotations per wheel steer rotation
                                   1.0/WHEEL_REVS_PER_ENC_REV,
                                   1.0/AZMTH_REVS_PER_ENC_REV, // same as motor rotations because NEO encoder is on motor shaft
                                   1.0/WHEEL_REVS_PER_ENC_REV,
                                   1.5,
                                   2,
                                   ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES, 
                                   0.01 
                                   );
    }
    

    public void resetRelativeRotationEncoders() {
        /* Note that we use the class name not a variable name.
         * This way we pass a method of the general SwerveModule class to be called
         * as each module.
         * So this expands to:
         *  modules.get(0).initRotationOffset();
         *  modules.get(1).initRotationOffset();
         *  ...
         */
        m_modules.forEach(SwerveModule::initRotationOffset);
    }

    public void resetPID() {
        m_xController.reset();
        m_yController.reset();
        m_thetaController.reset();
    }

    public Pose2d getTargetPose() {
        return new Pose2d(m_xController.getSetpoint(), m_yController.getSetpoint(), new Rotation2d(m_thetaController.getSetpoint()));
    }

    /****COMMANDS */
    public Command driveTime(double speed, double time) {
        return run(
            ()->driveFieldRelative(
                new ChassisSpeeds(speed, 0, 0)
            )
        )
        .withTimeout(time);
    }

    public Command turnToHeading(Rotation2d heading) {
        return run(()->{
            driveFieldRelative(
                new ChassisSpeeds(
                    0, 0, 
                    m_thetaController.calculate(
                        getPoseHeading().getRadians(),
                        heading.getRadians()
                    )
                )
            );
        });
    }

    public Command stopOnceC() {
        return runOnce(()->this.drive(new ChassisSpeeds()));
    }

    public Command stopC() {
        return run(()->this.drive(new ChassisSpeeds()));
    }
    public Command pathPlannerCommand(PathPlannerTrajectory path) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            path,
            this::getPose,
            m_xController,
            m_yController,
            m_thetaController,
            
            this::drive,
            true,
            this
        );
        return command;
    }

    /**
     * For use with PPChasePoseCommand
     * Generates a PathPlannerTrajectory on the fly to drive to the target pose.
     * Takes into account the current speed of the robot for the start point.
     * The returned PathPlannerTrajectory will go straight towards the target from the robot pose.
     * The component of the current velocity that points toward the target will be used as the initial
     * velocity of the trajectory.
     * @param robotPose the current robot pose
     * @param target the target pose
     * @param currentSpeedVectorMPS a Translation2d where x and y are the robot's x and y field-relative speeds in m/s.
     * @return a PathPlannerTrajectory to the target pose.
     */
    public static PathPlannerTrajectory generateTrajectoryToPose(Pose2d robotPose, Pose2d target, Translation2d currentSpeedVectorMPS, PathConstraints constraints) {

                
                // Robot velocity calculated from module states.
                Rotation2d fieldRelativeTravelDirection = NomadMathUtil.getDirection(currentSpeedVectorMPS);
                double travelSpeed = currentSpeedVectorMPS.getNorm();

                
                Translation2d robotToTargetTranslation = target.getTranslation().minus(robotPose.getTranslation());
                // Initial velocity override is the component of robot velocity along the robot-to-target vector.
                // If the robot velocity is pointing away from the target, start at 0 velocity.
                Rotation2d travelOffsetFromTarget = NomadMathUtil.getDirection(robotToTargetTranslation).minus(fieldRelativeTravelDirection);
                travelSpeed = Math.max(0, travelSpeed * travelOffsetFromTarget.getCos());
                // We only want to regenerate if the target is far enough away from the robot. 
                // PathPlanner has issues with near-zero-length paths and we need a particular tolerance for success anyway.
                if (
                    robotToTargetTranslation.getNorm() > 0.1
                ) {
                    PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.generatePath(
                        constraints, 
                        //Start point. At the position of the robot, initial travel direction toward the target,
                        // robot rotation as the holonomic rotation, and putting in the (possibly 0) velocity override.
                        new PathPoint(
                            robotPose.getTranslation(),
                            NomadMathUtil.getDirection(robotToTargetTranslation),
                            robotPose.getRotation(),
                            travelSpeed), // position, heading
                        // position, heading
                        new PathPoint(
                            target.getTranslation(),
                            NomadMathUtil.getDirection(robotToTargetTranslation),
                            target.getRotation()) // position, heading
                    );
                    return pathPlannerTrajectory;
                }

                return new PathPlannerTrajectory();
    }

    /**
     * Creates a new pose-chase command. 
     * This command generates and follows the target pose supplied by targetSupplier.
     * If the target has moved since the last generation, regen the trajectory.
     * If the trajectory is finished, switch to direct x-y-theta PID to hold the pose.
     * @param targetSupplier the Supplier for the target Pose2d.
     * @return the PPChasePoseCommand
     */
    public Command chasePoseC(Supplier<Pose2d> targetSupplier) {
        return new PPChasePoseCommand(
            targetSupplier,
            this::getPose,
            m_holonomicDriveController,
            this::drive,
            (PathPlannerTrajectory traj) -> {}, // empty output for current trajectory.
            (startPose, endPose)->DrivebaseS.generateTrajectoryToPose(startPose, endPose, getFieldRelativeLinearSpeedsMPS(), new PathConstraints(2, 2)),
            this);
    }

    public Command chargeStationAlignC() {
        return new PPChasePoseCommand(
            ()->new Pose2d(
                POIS.CHARGE_STATION.ownPose().getX(),
                getPose().getY(),
                POIS.CHARGE_STATION.ownPose().getRotation()),
            this::getPose,
            m_holonomicDriveController,
            this::drive,
            (PathPlannerTrajectory traj) -> {}, // empty output for current trajectory.
            (startPose, endPose)->DrivebaseS.generateTrajectoryToPose(startPose, endPose, getFieldRelativeLinearSpeedsMPS(),
                new PathConstraints(2, 3)),
            this);
    }

    public Command chargeStationOverAlignC() {
        return new PPChasePoseCommand(
            ()->new Pose2d(
                POIS.CHARGE_STATION_OVER.ownPose().getX(),
                getPose().getY(),
                POIS.CHARGE_STATION_OVER.ownPose().getRotation()),
            this::getPose,
            m_holonomicDriveController,
            this::drive,
            (PathPlannerTrajectory traj) -> {}, // empty output for current trajectory.
            (startPose, endPose)->DrivebaseS.generateTrajectoryToPose(startPose, endPose, getFieldRelativeLinearSpeedsMPS(),
                new PathConstraints(2, 3)),
            this);
    }
    public Command chargeStationBatteryFirstC() {
        return Commands.sequence(
            
            // high speed to push down the ramp (until a tilt is detected)
            run(()->this.driveAllianceRelative(new ChassisSpeeds(2, 0, 0)))
            .until(()->Math.abs(this.getPitch()) > 0.13).withTimeout(3),
            Commands.either(
                Commands.sequence(
                                // higher speed to get all the way on the ramp (for time)
                run(()->this.driveAllianceRelative(new ChassisSpeeds(1.5, 0, 0))).withTimeout(0.7),
                // slow speed to move past the tipping point (until it tips bac)
                run(()->this.driveAllianceRelative(new ChassisSpeeds(0.7, 0, 0)))
                    .alongWith(Commands.run(()->LightS.getInstance().requestState(States.Climbing)))
                    .until(()-> Math.abs(this.getPitch()) < 0.15),
                // short burst of backwards speed to cancel forward momentul
                run(()->this.driveAllianceRelative(new ChassisSpeeds(-0.6, 0, 0))).withTimeout(1),
                // put wheels in circle formation to prevent sliding
                run(()->this.driveAllianceRelative(new ChassisSpeeds(0, 0, 0.1))).withTimeout(0.2))
                , Commands.none(), ()->Math.abs(this.getPitch()) > 0.05)

        );
    }

    public Command chargeStationFrontFirstC() {
        return Commands.sequence(
            // high speed to push down the ramp (until a tilt is detected)
            run(()->this.drive(new ChassisSpeeds(1.3, 0, 0)))
            .until(()->Math.abs(this.getPitch()) > 0.13).withTimeout(3),
            Commands.either(
                Commands.sequence(
                                // higher speed to get all the way on the ramp (for time)
                run(()->this.drive(new ChassisSpeeds(1.5, 0, 0))).withTimeout(0.7),
                // slow speed to move past the tipping point (until it tips bac)
                run(()->this.drive(new ChassisSpeeds(0.7, 0, 0)))
                    .alongWith(Commands.run(()->LightS.getInstance().requestState(States.Climbing)))
                    .until(()-> Math.abs(this.getPitch()) < 0.15),
                // short burst of backwards speed to cancel forward momentul
                run(()->this.drive(new ChassisSpeeds(-0.6, 0, 0))).withTimeout(0.6),
                // put wheels in circle formation to prevent sliding
                run(()->this.drive(new ChassisSpeeds(0, 0, 0.1))).withTimeout(0.2))
                , Commands.none(), ()->Math.abs(this.getPitch()) > 0.05)

        );
    }

    public void scheduleConfigCommands() {
        m_modules.forEach(SwerveModule::scheduleConfigCommands);
    }

}