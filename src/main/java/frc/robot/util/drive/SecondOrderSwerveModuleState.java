package frc.robot.util.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderSwerveModuleState extends SwerveModuleState {

    public double omegaRadiansPerSecond;
    public double accelerationMetersPerSecondSquared;

    public SecondOrderSwerveModuleState() {}

    public SecondOrderSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double angularVelocity, double accelerationMetersPerSecondSquared) {
        super(speedMetersPerSecond, angle);
        this.omegaRadiansPerSecond = angularVelocity;
        this.accelerationMetersPerSecondSquared = accelerationMetersPerSecondSquared;
    }

    @Override
    public boolean equals(Object obj) {
        return super.equals(obj) && omegaRadiansPerSecond == omegaRadiansPerSecond && accelerationMetersPerSecondSquared == accelerationMetersPerSecondSquared;
    }

    @Override
    public int hashCode() {
        return super.hashCode() + Double.hashCode(omegaRadiansPerSecond) + Double.hashCode(accelerationMetersPerSecondSquared);
    }

    @Override
    public int compareTo(SwerveModuleState o) {
        return super.compareTo(o);
    }

    @Override
    public String toString() {
        return String.format(
            "SecondOrderSwerveModuleState(speedMetersPerSecond=%s, angle=%s, angularVelocity=%s, accelerationMetersPerSecondSquared=%s)",
            speedMetersPerSecond, angle, omegaRadiansPerSecond, accelerationMetersPerSecondSquared
        );
    }

    public static SecondOrderSwerveModuleState optimize(
      SecondOrderSwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SecondOrderSwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)),
          desiredState.omegaRadiansPerSecond,
          -desiredState.accelerationMetersPerSecondSquared);
    } else {
      return new SecondOrderSwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle,
        desiredState.omegaRadiansPerSecond, desiredState.accelerationMetersPerSecondSquared);
    }
  }

}