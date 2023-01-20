package frc.robot.util.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Arrays;
import java.util.Collections;
import org.ejml.simple.SimpleMatrix;

/**
 * This class is an implementation of {@code SwerveDriveKinematics} that uses a second order kinematics model.
 * This model is *supposed* to be more accurate than the original class since it uses accelerations as well as
 * velocities in its calculation and output of module states. (Supposed to :D )
 */
public class SecondOrderSwerveDriveKinematics extends SwerveDriveKinematics {
    //First Order Kinematics
    private SimpleMatrix m_firstOrderForward;
    private SimpleMatrix m_firstOrderInverse;

    //Second Order Kinematics
    private SimpleMatrix m_secondOrderForward;
    private SimpleMatrix m_secondOrderInverse;

    //Swerve Specifications and State
    private final int m_numModules;
    private final Translation2d[] m_modules;
    private Translation2d m_prevCoR = new Translation2d();

    /**
     * Constructs a swerve drive kinematics object.
     * @param wheelsMeters The locations of the wheels on the robot relative to the center of rotation.
     */
    public SecondOrderSwerveDriveKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters);
        System.out.println(wheelsMeters.length);
        if (wheelsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
          }

          //First Order Matrices

          m_numModules = wheelsMeters.length;
          m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
          m_firstOrderInverse = new SimpleMatrix(m_numModules * 2, 3);
      
          for (int i = 0; i < m_numModules; i++) {
            m_firstOrderInverse.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
            m_firstOrderInverse.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
          }
          m_firstOrderForward = m_firstOrderInverse.pseudoInverse();
      
          MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);



          // Second Order Matrices
            m_secondOrderInverse = new SimpleMatrix(m_numModules * 2, 4);
          for(int i = 0; i < m_numModules; i++) {
              m_secondOrderInverse.setRow(i*2+0, 1, 0, -m_modules[i].getX(), -m_modules[i].getY());
              m_secondOrderInverse.setRow(i*2+1, 0, 1, -m_modules[i].getY(), m_modules[i].getX());
          }

          m_secondOrderForward = m_secondOrderInverse.pseudoInverse();
    }

    /**
     * 
     * Takes in desired chassis speeds (with desired accelerations) and returns the module states required to achieve those speeds (with accelerations)
     * 
     * @param chassisSpeeds The desired chassis speeds.
     * @param centerOfRotationMeters The center of rotation of the robot.
     * @return The module states required to achieve the desired chassis speeds.
     */
    public SecondOrderSwerveModuleState[] toSwerveModuleStates(SecondOrderChassisSpeeds chassisSpeeds, 
        Translation2d centerOfRotationMeters) {

            if(!centerOfRotationMeters.equals(m_prevCoR)) {
                for(int i = 0; i < m_numModules; i++) {
                    m_firstOrderInverse.setRow(i*2+0, 0, /* Start Data */ 1, 0, -m_modules[i].getY() + centerOfRotationMeters.getY());
                    m_firstOrderInverse.setRow(i*2+1, 0, /* Start Data */ 0, 1, +m_modules[i].getX() - centerOfRotationMeters.getX());
                }
                // m_firstOrderForward = m_firstOrderInverse.pseudoInverse();

                for(int i = 0; i < m_numModules; i++) {
                    m_secondOrderInverse.setRow(i*2+0, 0, 1, 0, -m_modules[i].getX() + centerOfRotationMeters.getX(), -m_modules[i].getY() + centerOfRotationMeters.getY());
                    m_secondOrderInverse.setRow(i*2+1, 0, 0, 1, -m_modules[i].getY() + centerOfRotationMeters.getY(), m_modules[i].getX() - centerOfRotationMeters.getX());
                }
                // m_secondOrderForward = m_secondOrderInverse.pseudoInverse();

                m_prevCoR = centerOfRotationMeters;
            }

            //First Order Kinematics

            var chassisSpeedsVector = new SimpleMatrix(3, 1);
            chassisSpeedsVector.setColumn(
            0,
          0,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);

            var moduleStatesMatrix = m_firstOrderInverse.mult(chassisSpeedsVector);

            SwerveModuleState[] firstOrderStates = new SwerveModuleState[m_numModules];

            for (int i = 0; i < m_numModules; i++) {
                double x = moduleStatesMatrix.get(i * 2, 0);
                double y = moduleStatesMatrix.get(i * 2 + 1, 0);

                double speed = Math.hypot(x, y);
                Rotation2d angle = new Rotation2d(x, y);

                firstOrderStates[i] = new SwerveModuleState(speed, angle);
            }

            //Second Order Kinematics

            var secondOrderChassisVector = new SimpleMatrix(4, 1);
            secondOrderChassisVector.setColumn(
                0,
                0,
                chassisSpeeds.axMetersPerSecondSq,
                chassisSpeeds.ayMetersPerSecondSq,
                chassisSpeeds.omegaRadiansPerSecond*chassisSpeeds.omegaRadiansPerSecond,
                chassisSpeeds.alphaRadiansPerSecondSq
            );

            var secondOrderModuleStatesMatrix = m_secondOrderInverse.mult(secondOrderChassisVector);

            SecondOrderSwerveModuleState[] secondOrderStates = new SecondOrderSwerveModuleState[m_numModules];

            for(int i = 0; i < m_numModules; i++) {
                double moduleAngle = firstOrderStates[i].angle.getRadians();
                var angleMatrix = new SimpleMatrix(2, 2);
                angleMatrix.setRow(0, 0, Math.cos(moduleAngle), Math.sin(moduleAngle));
                angleMatrix.setRow(1, 0, -Math.sin(moduleAngle), Math.cos(moduleAngle));

                var secondOrderFinal = angleMatrix.mult(secondOrderModuleStatesMatrix.extractMatrix(i*2, i*2+2, 0, 1));

                double moduleAcceleration = secondOrderFinal.get(0, 0);
                double moduleAngularVelocity = secondOrderFinal.get(1, 0) / firstOrderStates[i].speedMetersPerSecond;
                
                secondOrderStates[i] = new SecondOrderSwerveModuleState(firstOrderStates[i].speedMetersPerSecond, firstOrderStates[i].angle, moduleAngularVelocity, moduleAcceleration);
            }


            return secondOrderStates;
    }

    /**
     * 
     * This will return the necessary second order swerve module states to achieve the desired chassis speeds.
     * The method assumes that the center of rotation is the same as was used in the previous call to this object.
     * 
     * @param chassisSpeeds The desired chassis speeds.
     * @return The module states required to achieve the desired chassis speeds.
     */
    public SecondOrderSwerveModuleState[] toSwerveModuleStates(SecondOrderChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }


    /**
     * 
     * This is an experimental implementation of how to determine what translational accelerations the robot should have.
     * The way it works is that it takes in the desired chassis speed ({@code chassisSpeeds}) and the chassis speed
     * that is calculated by the odometry ({@code chassisSpeedsPrev}). It then calculates the desired acceleration
     * of the robot as the necessary acceleration to get from the current speeds (as per the odometry) to the desired speeds in a time
     * period of {@code dt}.
     * 
     * To get the best estimation for {@code chassisSpeedsPrev}, the current chassis speeds of the robot,
     * you should probably use the regular toChassisSpeeds method in {@code SwerveDriveKinematics}
     * 
     * Example:
     * swerveKinematics.toChassisSpeeds(getStates()); (in Swerve.java)
     * 
     * @param chassisSpeeds The desired chassis speeds of the robot (first order, so no accelerations)
     * @param chassisSpeedsPrev The current chassis speeds of the robot (first order, so no accelerations)
     * @param centerOfRotationMeters The center of rotation of the robot
     * @param dt The time period over which the robot should accelerate
     * @return The second order states of the modules (so with accelerations)
     */
    public SecondOrderSwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, ChassisSpeeds chassisSpeedsPrev, Translation2d centerOfRotationMeters, double dt) {

        //Create a SecondOrderChassisSpeeds object from the two ChassisSpeeds objects
        //Use the velocities from the chassisSpeeds object and calculate the accelerations from the chassisSpeeds and chassisSpeedsPrev objects
        //Call the toSwerveModuleStates(SecondOrderChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) method

        SecondOrderChassisSpeeds secondOrderChassisSpeeds = new SecondOrderChassisSpeeds(chassisSpeeds.vxMetersPerSecond, 
        chassisSpeeds.vyMetersPerSecond, 
        chassisSpeeds.omegaRadiansPerSecond, 
        (chassisSpeeds.vxMetersPerSecond - chassisSpeedsPrev.vxMetersPerSecond) / dt, 
        (chassisSpeeds.vyMetersPerSecond - chassisSpeedsPrev.vyMetersPerSecond) / dt, 
        (chassisSpeeds.omegaRadiansPerSecond - chassisSpeedsPrev.omegaRadiansPerSecond) / dt);

        return toSwerveModuleStates(secondOrderChassisSpeeds, centerOfRotationMeters);
    }

    public SecondOrderChassisSpeeds toChassisSpeeds(SecondOrderSwerveModuleState... wheelStates) {

        if(wheelStates.length != m_numModules) {
            throw new IllegalArgumentException("The number of wheel states must be equal to the number of modules");
        }

        //First Order

        var moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            var module = wheelStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
          }
      
          var chassisSpeedsVector = m_firstOrderForward.mult(moduleStatesMatrix);

          //Second Order

        var accelerationAxisVector = new SimpleMatrix(2*m_numModules, 1);

        for(int i = 0; i < m_numModules; i++) {
            var secondOrderVector = new SimpleMatrix(2, 1);
            secondOrderVector.set(0, 0, wheelStates[i].accelerationMetersPerSecondSquared);
            secondOrderVector.set(1, 0, wheelStates[i].omegaRadiansPerSecond * wheelStates[i].speedMetersPerSecond);

            double moduleAngle = wheelStates[i].angle.getRadians();
            var angleMatrix = new SimpleMatrix(2, 2);

            angleMatrix.setRow(0, 0, Math.cos(moduleAngle), Math.sin(moduleAngle));
            angleMatrix.setRow(1, 0, -Math.sin(moduleAngle), Math.cos(moduleAngle));

            var angleMatrixInv = angleMatrix.invert();

            var secondOrderVectorFinal = angleMatrixInv.mult(secondOrderVector);

            accelerationAxisVector.set(i*2, 0, secondOrderVectorFinal.get(0, 0));
            accelerationAxisVector.set(i*2+1, 0, secondOrderVectorFinal.get(1, 0));
        }

        var chassisAccelerationVector = m_secondOrderForward.mult(accelerationAxisVector);

        double v_x = chassisSpeedsVector.get(0, 0);
        double v_y = chassisSpeedsVector.get(1, 0);
        double omega = chassisSpeedsVector.get(2, 0);

        double a_x = chassisAccelerationVector.get(0, 0);
        double a_y = chassisAccelerationVector.get(1, 0);
        double otherOmega = Math.sqrt(chassisAccelerationVector.get(2,0));
        double alpha = chassisAccelerationVector.get(3, 0);


        SecondOrderChassisSpeeds secondOrderChassisSpeeds = new SecondOrderChassisSpeeds(v_x, v_y, omega, a_x, a_y, alpha);


        return secondOrderChassisSpeeds;
    }

    public static void desaturateWheelSpeeds(
        SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
      double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;
      if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
        for (SwerveModuleState moduleState : moduleStates) {
          moduleState.speedMetersPerSecond =
              moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
        }
      }
    }


}