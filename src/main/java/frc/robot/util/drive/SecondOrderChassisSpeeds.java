package frc.robot.util.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SecondOrderChassisSpeeds extends ChassisSpeeds {
    
    public double axMetersPerSecondSq;
    public double ayMetersPerSecondSq;
    public double alphaRadiansPerSecondSq;

    public SecondOrderChassisSpeeds() {}

    public SecondOrderChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this(chassisSpeeds, 0, 0, 0);

    }

    public SecondOrderChassisSpeeds(ChassisSpeeds chassisSpeeds,
        double chassisAccelerationXMetersPerSecond,
        double chassisAccelerationYMetersPerSecond,
        double chassisAccelerationAlphaRadiansPerSecond) {
        this(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
            chassisAccelerationXMetersPerSecond,
            chassisAccelerationYMetersPerSecond,
            chassisAccelerationAlphaRadiansPerSecond);
    }

    public SecondOrderChassisSpeeds(double vxMetersPerSecond, double vymetersPerSecond, double omegaRadiansPerSecond, double axMetersPerSecond, double ayMetersPerSecond, double alphaRadiansPerSecondSquared) {
        super(vxMetersPerSecond, vymetersPerSecond, omegaRadiansPerSecond);
        this.axMetersPerSecondSq = axMetersPerSecond;
        this.ayMetersPerSecondSq = ayMetersPerSecond;
        this.alphaRadiansPerSecondSq = alphaRadiansPerSecondSquared;
    }

    public static SecondOrderChassisSpeeds fromFieldRelativeSpeeds(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double axMetersPerSecond,
    double ayMetersPerSecond,
    double alphaRadiansPerSecondSquared,
      Rotation2d robotAngle) {

        return new SecondOrderChassisSpeeds(
            vxMetersPerSecond * robotAngle.getCos() + vyMetersPerSecond * robotAngle.getSin(),
            -vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(),
            omegaRadiansPerSecond,
            axMetersPerSecond * robotAngle.getCos() + ayMetersPerSecond * robotAngle.getSin(),
            -axMetersPerSecond * robotAngle.getSin() + ayMetersPerSecond * robotAngle.getCos(),
            alphaRadiansPerSecondSquared
        );

      }

      @Override
      public String toString() {
        return String.format(
            "SecondOrderChassisSpeeds(vxMetersPerSecond=%s, vyMetersPerSecond=%s, omegaRadiansPerSecond=%s, axMetersPerSecond=%s, ayMetersPerSecond=%s, alphaRadiansPerSecondSquared=%s)",
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, axMetersPerSecondSq, ayMetersPerSecondSq, alphaRadiansPerSecondSq
        );
      }

}
