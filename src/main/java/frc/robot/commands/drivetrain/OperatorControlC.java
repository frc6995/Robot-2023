package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.util.drive.SecondOrderChassisSpeeds;

public class OperatorControlC extends CommandBase {

    /**
     * Command to allow for driver input in teleop
     * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
     */

    private final DrivebaseS m_drive;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
    private final DoubleSupplier m_forwardX;
    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(3);
    private final DoubleSupplier m_forwardY;
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(3);
    private final DoubleSupplier m_rotation;
    private final SlewRateLimiter m_thetaRateLimiter = new SlewRateLimiter(2);

    private final double MAX_LINEAR_SPEED = Units.feetToMeters(8);

    public static final double MAX_TURN_SPEED = Units.degreesToRadians(360);

    public OperatorControlC(
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier rot,
        DrivebaseS subsystem
    ) {

        m_drive = subsystem;
        m_forwardX = fwdX;
        m_forwardY = fwdY;
        m_rotation = rot;

        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        m_xRateLimiter.reset(0);
        m_yRateLimiter.reset(0);
        m_thetaRateLimiter.reset(0);
    }
    
    @Override
    public void execute() {
        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */

        double fwdX = -m_forwardX.getAsDouble();
        fwdX = Math.copySign(fwdX, fwdX);
        fwdX = deadbandInputs(fwdX);

        double fwdY = -m_forwardY.getAsDouble();
        fwdY = Math.copySign(fwdY, fwdY);
        fwdY = deadbandInputs(fwdY);

        double driveDirectionRadians = Math.atan2(fwdY, fwdX);
        double driveMagnitude = Math.hypot(fwdX, fwdY) * MAX_LINEAR_SPEED;
        fwdX = driveMagnitude * Math.cos(driveDirectionRadians);
        fwdY = driveMagnitude * Math.sin(driveDirectionRadians);

        double rot = -m_rotation.getAsDouble();
        //rot = Math.copySign(rot * rot, rot);
        rot = deadbandInputs(rot);
        rot = m_thetaRateLimiter.calculate(rot);
        rot *= MAX_TURN_SPEED;

        m_drive.drive(SecondOrderChassisSpeeds.fromFieldRelativeSpeeds(
            new SecondOrderChassisSpeeds(
                new ChassisSpeeds(fwdX, fwdY, rot)
            ),
            //Fudge factor here
            m_drive.getPoseHeading().plus(Rotation2d.fromRadians(rot * 0.09))
        ));
    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < 0.2) return 0.0;
        return input;

    }

}