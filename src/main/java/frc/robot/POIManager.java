package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.AllianceWrapper;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

public class POIManager {
    // 9 scoring positions
    // be able to flip each position to other alliance
    // 6 arm positions (3 rows x 2 pieces)
    public static final double FIELD_LENGTH = 16.542;
    public static final double FIELD_WIDTH = 8.014;
    public enum POIS {
        CUBE_RAMP (POIManager.mirrorPose(new Pose2d(2.056, 7.484, Rotation2d.fromDegrees(90)))),
        CONE_RAMP (new Pose2d(14.511, 7.296, Rotation2d.fromDegrees(90))),
        GRID_PLAT (new Pose2d(15.81, 6.05, new Rotation2d())),
        WALL_PLAT (new Pose2d(15.81, 7.419, new Rotation2d())),
        CHARGE_STATION(new Pose2d(3.9, 2.74, Rotation2d.fromDegrees(180)),
            POIManager.mirrorPose(new Pose2d(3.9, 2.74, Rotation2d.fromDegrees(180)))),
        CHARGE_STATION_OVER(new Pose2d(5.81, 2.74, Rotation2d.fromDegrees(180)));
        

        Pose2d bluePose;
        Pose2d redPose;
        private POIS(Pose2d bluePose) {
            this.bluePose = bluePose;
            redPose = POIManager.mirrorPose(bluePose);

        }

        private POIS(Pose2d bluePose, Pose2d redPose) {
            this.bluePose = bluePose;
            this.redPose = redPose;

        }


        public Pose2d ownPose() {
            if (AllianceWrapper.getAlliance() == Alliance.Red) {
                return redPose;
            }
            else {
                return bluePose;
            }
        }
    }
    /**
     * The drivetrain scoring positions for the blue community.
     * Pose 0 is for the wall side, the farthest-right scoring station from the driver perspective. 
     */
    public static final List<Pose2d> BLUE_COMMUNITY = List.of(
        new Pose2d(1.843, 0.547, fromDegrees(180)),
        new Pose2d(1.843, 1.068, fromDegrees(180)),
        new Pose2d(1.843, 1.628, fromDegrees(180)),
        new Pose2d(1.843, 2.176, fromDegrees(180)),
        new Pose2d(1.843, 2.736, fromDegrees(180)),
        new Pose2d(1.843, 3.296, fromDegrees(180)),
        new Pose2d(1.843, 3.870, fromDegrees(180)),
        new Pose2d(1.843, 4.417, fromDegrees(180)),
        new Pose2d(1.843, 4.983, fromDegrees(180))
    );

    /**
     * mirrors the Pose values of the blue community
     */

    public static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getTranslation().getX(), pose.getTranslation().getY(),
            fromDegrees(180).minus(pose.getRotation()));
    }

    public static Pose2d mirrorPoseAlliance(Pose2d bluePose) {
        if (AllianceWrapper.getAlliance() == Alliance.Blue) {
            return bluePose;
        }
        else {
            return mirrorPose(bluePose);
        }

    }

    /**
     * Drivebase scoring positions for the red community.
     * Pose 0 is the farthest-right scoring position from the driver's pov.
     * Pose values are mirrored from the blue community.
     */

    public static final List<Pose2d> RED_COMMUNITY;
    public static final List<Pose2d> RED_FIELD_COMMUNITY;
    static {
        List<Pose2d> mirroredCommunity = new ArrayList<Pose2d>();
        List<Pose2d> mirroredFieldCommunity = new ArrayList<Pose2d>();
        for (int i = BLUE_COMMUNITY.size() - 1; i >= 0; i--) {
            mirroredCommunity.add(mirrorPose(BLUE_COMMUNITY.get(i)));
            mirroredFieldCommunity.add(mirrorPose(BLUE_COMMUNITY.get(BLUE_COMMUNITY.size()-1-i)));
        }
        RED_COMMUNITY = mirroredCommunity;
        RED_FIELD_COMMUNITY = mirroredFieldCommunity;

    }

    public static List<Pose2d> ownCommunity() {
        if (AllianceWrapper.getAlliance() == Alliance.Red) {
            return RED_COMMUNITY;
        }
        else {
            return BLUE_COMMUNITY;
        }
    }
    public static List<Pose2d> ownCommunityFlipped() {
        if (AllianceWrapper.getAlliance() == Alliance.Red) {
            return RED_FIELD_COMMUNITY;
        }
        else {
            return BLUE_COMMUNITY;
        }
    }
    public static Pose2d ownPOI(POIS poi) {
        if (AllianceWrapper.getAlliance() == Alliance.Red) {
            return poi.redPose;
        }
        else {
            return poi.bluePose;
        }
    }


}
