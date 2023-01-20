package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {

    public static final class InputDevices {

        public static final int GAMEPAD_PORT = 0;

    }

    public static final class DriveConstants {

        static public final double WHEEL_BASE_WIDTH_M = Units.inchesToMeters(18.25);
        static public final double WHEEL_RADIUS_M = 0.0508; //Units.inchesToMeters(4.0/2.0); //four inch (diameter) wheels
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(35);
        static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_WIDTH_M*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center
        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(19.0);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(19.0);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(720.0);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.125; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second
        
    // HELPER ORGANIZATION CONSTANTS
        static public final int FL = 0; // Front Left Module Index
        static public final int FR = 1; // Front Right Module Index
        static public final int BL = 2; // Back Left Module Index
        static public final int BR = 3; // Back Right Module Index
        static public final int NUM_MODULES = 4;

        // Internal objects used to track where the modules are at relative to
        // the center of the robot, and all the implications that spacing has.
        static private double HW = WHEEL_BASE_WIDTH_M/2.0;

        public enum ModuleConstants {
            FL("FL", 18, 17, 6, 5.708331, HW, HW),
            FR("FR", 12, 11, 7, 3.783000, HW, -HW),
            BL("BL", 16, 15, 8, 1.787997, -HW, HW),
            BR("BR", 14, 13, 9, 5.660000, -HW, -HW);
    
            public final String name;
            public final int driveMotorID;
            public final int rotationMotorID;
            public final int magEncoderID;
            /**
             * absolute encoder offsets for the wheels
             * 180 degrees added to offset values to invert one side of the robot so that it doesn't spin in place
             */
            public final double magEncoderOffset;
            public final Translation2d centerOffset;
            private ModuleConstants(String name, int driveMotorID, int rotationMotorID, int magEncoderID, double magEncoderOffset, double xOffset, double yOffset) {
                this.name = name;
                this.driveMotorID = driveMotorID;
                this.rotationMotorID = rotationMotorID;
                this.magEncoderID = magEncoderID;
                this.magEncoderOffset = magEncoderOffset;
                centerOffset = new Translation2d(xOffset, yOffset);
    
            }
        }
        
        public static final double WHEEL_REVS_PER_ENC_REV = 1.0/5.14;
        public static final double AZMTH_REVS_PER_ENC_REV = 1.0/12.8;

        public static final double STEER_MAX_SPEED_RAD_PER_SEC = 7.8 * 2 * Math.PI;
        public static final double STEER_MAX_ACCEL_RAD_PER_SEC_SQ = 400 * 2 * Math.PI;

        //kv: (12 volts * 60 s/min * 1/5.14 WRevs/MRevs * wheel rad * 2pi  / (6000 MRPM *
        /** ks, kv, ka */ 
        public static final double[] DRIVE_FF_CONST = {0.11452, 1.9844, 0.31123 * 1.2};

        public static final double STEER_P = 2.5;
        public static final double STEER_D = 0.0;
    
        public static final double DRIVE_P = 0.75;//9;
        public static final double DRIVE_D = 0.0;


        public static final double MAX_MODULE_SPEED_FPS = 19;

        public static final int ENC_PULSE_PER_REV = 1;
        public static final double WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV/ WHEEL_REVS_PER_ENC_REV;  //Assume 1-1 gearing for now
        public static final double AZMTH_ENC_COUNTS_PER_MODULE_REV = ENC_PULSE_PER_REV / AZMTH_REVS_PER_ENC_REV; //Assume 1-1 gearing for now
        public static final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        public static final double AZMTH_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(AZMTH_ENC_COUNTS_PER_MODULE_REV));

        public static final TrapezoidProfile.Constraints X_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
        public static final TrapezoidProfile.Constraints Y_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);

        public static final TrapezoidProfile.Constraints NO_CONSTRAINTS = new TrapezoidProfile.Constraints(Integer.MAX_VALUE, Integer.MAX_VALUE);
        public static final TrapezoidProfile.Constraints THETA_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(4*Math.PI, 16*Math.PI);
    

    }

    public static final class VisionConstants{

        public static final Transform3d robotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.25), 0, Units.inchesToMeters(32.5)), 
            new Rotation3d());
        public static AprilTagFieldLayout TAG_FIELD_LAYOUT = new AprilTagFieldLayout(
            List.of(
                new AprilTag(
                        1,
                        new Pose3d(
                        15.513558, 1.071626, 0.462788, new Rotation3d(0, 0, Math.PI))
                        ),
                new AprilTag(
                        2,
                        new Pose3d(
                        15.513558, 2.748026, 0.462788, new Rotation3d(0, 0, Math.PI))
                        ),
                new AprilTag(
                        3,
                        new Pose3d(
                        15.513558, 4.424426, 0.462788, new Rotation3d(0, 0, Math.PI))
                        ),
                new AprilTag(
                        4,
                        new Pose3d(
                        16.178784, 6.749796, 0.695452, new Rotation3d(0, 0, Math.PI))
                        ),
                new AprilTag(
                        5,
                        new Pose3d(
                        0.36195, 6.749796, 0.695452, new Rotation3d(0, 0, 0))
                        ),
                new AprilTag(
                        6,
                        new Pose3d(
                        1.02743, 4.424426, 0.462788, new Rotation3d(0, 0, 0))
                        ),
                new AprilTag(
                        7,
                        new Pose3d(
                        1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, 0))
                        ),
                new AprilTag(
                        8,
                        new Pose3d(
                        1.02743, 1.071626, 0.462788, new Rotation3d(0, 0, 0))
                        )




            ),
            16.54175,
            8.0137
        );
    }
}
