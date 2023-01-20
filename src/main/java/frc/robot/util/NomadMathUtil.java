package frc.robot.util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NomadMathUtil {

    public static Rotation2d getDirection(Transform2d transform) {
        return getDirection(transform.getTranslation());
    }

    public static Rotation2d getDirection(Translation2d transform) {
        //add tiny number so that 0/0 comes out to 0 angle, not a div by 0 error
        return new Rotation2d(transform.getX(), transform.getY());
    }

    public static Rotation2d getDirection(Pose2d tail, Pose2d head) {
        return getDirection(head.getTranslation().minus(tail.getTranslation()));
    }

    public static double getDistance(Transform2d transform){
        return getDistance(transform.getTranslation());
    }

    public static double getDistance(Translation2d transform) {
        return transform.getNorm();
    }


    public static double subtractkS(double voltage, double kS) {
        if(Math.abs(voltage) <= kS) {
            voltage = 0;
        }
        else {
            voltage -= Math.copySign(kS, voltage);
        }
        return voltage;
    }
 
      public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle, double flipThreshold) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > flipThreshold) {
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
  }

    
}
