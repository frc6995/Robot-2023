package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.drive.AsymmetricSlewRateLimiter;
import frc.robot.util.drive.SecondOrderChassisSpeeds;

public class OperatorControlHeadingC extends CommandBase {

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
    private final AsymmetricSlewRateLimiter m_xRateLimiter = new AsymmetricSlewRateLimiter(1, 3);
    private final DoubleSupplier m_forwardY;
    private final AsymmetricSlewRateLimiter m_yRateLimiter = new AsymmetricSlewRateLimiter(1, 3);
    private final DoubleSupplier m_heading;

    private final double MAX_LINEAR_SPEED = Units.feetToMeters(11);

    public OperatorControlHeadingC(
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier heading,
        DrivebaseS subsystem
    ) {

        m_drive = subsystem;
        m_forwardX = fwdX;
        m_forwardY = fwdY;
        m_heading = heading;

        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        m_xRateLimiter.reset(preprocessInputs(-m_forwardX.getAsDouble()));
        m_yRateLimiter.reset(preprocessInputs(-m_forwardY.getAsDouble()));
        m_drive.m_profiledThetaController.reset(m_drive.getPoseHeading().getRadians(), 0);
    }

    private double preprocessInputs (double input) {
        input = deadbandInputs(input);
        input = Math.copySign(input * input , input);
        return input;
    }
    
    @Override
    public void execute() {
        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */

        double fwdX = -m_forwardX.getAsDouble();
        fwdX = preprocessInputs(fwdX);

        fwdX = m_xRateLimiter.calculate(fwdX);
        

        double fwdY = -m_forwardY.getAsDouble();
        fwdY = preprocessInputs(fwdY);
        fwdY = m_yRateLimiter.calculate(fwdY);
        

        double driveDirectionRadians = Math.atan2(fwdY, fwdX);
        double driveMagnitude = Math.hypot(fwdX, fwdY) * MAX_LINEAR_SPEED;
        fwdX = driveMagnitude * Math.cos(driveDirectionRadians);
        fwdY = driveMagnitude * Math.sin(driveDirectionRadians);

        var rot = m_drive.m_profiledThetaController.calculate(m_drive.getPoseHeading().getRadians(), m_heading.getAsDouble());

        var correctedHeading = m_drive.getPoseHeading().plus(Rotation2d.fromRadians(rot * 0.09));
        if (AllianceWrapper.getAlliance() == Alliance.Red) {
            correctedHeading = correctedHeading.plus(Rotation2d.fromRadians(Math.PI));
        }
        m_drive.drive(SecondOrderChassisSpeeds.fromFieldRelativeSpeeds(
            fwdX, fwdY, rot, 0, 0, 0,
            //Fudge factor here
            correctedHeading
        ));
    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < 0.2) return 0.0;
        return input;

    }

}