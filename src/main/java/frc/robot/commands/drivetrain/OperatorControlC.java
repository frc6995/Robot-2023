package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.drive.AsymmetricSlewRateLimiter;

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
    private final AsymmetricSlewRateLimiter m_xRateLimiter = new AsymmetricSlewRateLimiter(2, 6);
    private final DoubleSupplier m_forwardY;
    private final AsymmetricSlewRateLimiter m_yRateLimiter = new AsymmetricSlewRateLimiter(2, 6);
    private final DoubleSupplier m_rotation;
    private final SlewRateLimiter m_thetaRateLimiter = new SlewRateLimiter(2);
    private final DoubleSupplier m_headingToHold;
    private final BooleanSupplier m_holdHeading;

    private boolean lastHoldHeading = false;

    private final double MAX_LINEAR_SPEED = Units.feetToMeters(14);

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
        m_headingToHold = ()->0;
        m_holdHeading = ()->false;

        addRequirements(subsystem);

    }

    public OperatorControlC(
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier rot,
        DoubleSupplier headingToHold,
        BooleanSupplier holdHeading,
        DrivebaseS subsystem
    ) {

        m_drive = subsystem;
        m_forwardX = fwdX;
        m_forwardY = fwdY;
        m_rotation = rot;
        m_headingToHold = headingToHold;
        m_holdHeading = holdHeading;

        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        double fwdX = -m_forwardX.getAsDouble();
        fwdX = deadbandInputs(fwdX);
        fwdX = Math.copySign(fwdX*fwdX, fwdX);
        m_xRateLimiter.reset(fwdX);

        double fwdY = -m_forwardY.getAsDouble();
        fwdY = deadbandInputs(fwdY);
        fwdY = Math.copySign(fwdY*fwdY, fwdY);
        m_yRateLimiter.reset(fwdY);

        double rot = -m_rotation.getAsDouble();
        //rot = Math.copySign(rot * rot, rot);
        rot = deadbandInputs(rot);
        m_thetaRateLimiter.reset(rot);
        
    }
    
    @Override
    public void execute() {

        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */

        double fwdX = -m_forwardX.getAsDouble();
        fwdX = deadbandInputs(fwdX);
        fwdX = Math.copySign(fwdX*fwdX, fwdX);
        fwdX = m_xRateLimiter.calculate(fwdX);
        

        double fwdY = -m_forwardY.getAsDouble();
        fwdY = deadbandInputs(fwdY);
        fwdY = Math.copySign(fwdY*fwdY, fwdY);
        fwdY = m_yRateLimiter.calculate(fwdY);
        

        double driveDirectionRadians = Math.atan2(fwdY, fwdX);
        double driveMagnitude = Math.hypot(fwdX, fwdY) * MAX_LINEAR_SPEED;
        fwdX = driveMagnitude * Math.cos(driveDirectionRadians);
        fwdY = driveMagnitude * Math.sin(driveDirectionRadians);

        double rot;
        rot = -m_rotation.getAsDouble();
        //rot = Math.copySign(rot * rot, rot);
        rot = deadbandInputs(rot);
        rot = m_thetaRateLimiter.calculate(rot);
        if (!m_holdHeading.getAsBoolean()) {


            rot *= MAX_TURN_SPEED;
            lastHoldHeading = false;
        }
        else {
            if (!lastHoldHeading) {
                m_drive.m_profiledThetaController.reset(m_drive.getPoseHeading().getRadians(), 0);
            }
            rot = m_drive.m_profiledThetaController.calculate(m_drive.getPoseHeading().getRadians(), m_headingToHold.getAsDouble());
            lastHoldHeading = true;
        }

        var correctedHeading = m_drive.getPoseHeading().plus(Rotation2d.fromRadians(rot * 0.09));
        if (AllianceWrapper.getAlliance() == Alliance.Red) {
            correctedHeading = correctedHeading.plus(Rotation2d.fromRadians(Math.PI));
        }
        m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            fwdX, fwdY, rot,
            //Fudge factor here
            correctedHeading
        ));
    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        return MathUtil.applyDeadband(input, 0.2);

    }

}