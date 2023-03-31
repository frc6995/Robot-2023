package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmS.ArmPosition;

public class Constants {

    public static final class InputDevices {

        public static final int GAMEPAD_PORT = 0;

    }

    public static final class DriveConstants {
        static public final double WHEEL_BASE_WIDTH_M = Units.inchesToMeters(18.25);
        static public final double WHEEL_RADIUS_M = 0.0508; //Units.inchesToMeters(4.0/2.0); //four inch (diameter) wheels
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(150);
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
            FL("FL", 18, 17, 6, 5.651346 + 0.005, HW, HW),
            FR("FR", 12, 11, 7, 3.762759 - 0.003, HW, -HW),
            BL("BL", 16, 15, 8, 1.823007 + 0.01, -HW, HW),
            BR("BR", 14, 13, 9, 5.614791 - 0.008, -HW, -HW);
    
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
        public static final double[] DRIVE_FF_CONST = {0.14315, 2.1567, 0.57};

        public static final double STEER_P = 2.5;
        public static final double STEER_D = 0.0;
    
        public static final double DRIVE_P = 2;//9;
        public static final double DRIVE_D = 0.05;


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

        public static final String CAM_2_NAME = "OV9281-2";
        public static final String CAM_1_NAME = "OV9281-1";
        public static final Transform3d robotToCam2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(9.625), Units.inchesToMeters(8.75), Units.inchesToMeters(10.875)), 
            new Rotation3d(Units.degreesToRadians(0) , Units.degreesToRadians(-12),Units.degreesToRadians(-46) ));
        public static final Transform3d robotToCam1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(9.625), Units.inchesToMeters(-8.75), Units.inchesToMeters(10.875)), 
            new Rotation3d(Units.degreesToRadians(0) , Units.degreesToRadians(-12),Units.degreesToRadians(46) ));
        public static AprilTagFieldLayout TAG_FIELD_LAYOUT = 
        new AprilTagFieldLayout(
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

    public static final class ArmConstants {


        /* EXTEND */
        
        public static final int EXTEND_MOTOR_ID = 20;

        //Arm length measured from shoulder pivot to wrist pivot
        public static final double MIN_ARM_LENGTH = Units.inchesToMeters(19.875);
        public static final double MAX_ARM_LENGTH = 1.497;

        public static final Translation2d ARM_PIVOT_TRANSLATION = new Translation2d(0, Units.inchesToMeters(18.69));
        
        public static final double EXTEND_DRUM_RADIUS = 0.022238;
        public static final double EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION = 1.0/16.0;
        public static final double EXTEND_METERS_PER_DRUM_ROTATION = Math.PI * 2 * EXTEND_DRUM_RADIUS * 2; // 2x distance

        public static final double ARM_EXTEND_KG_VERTICAL = 0.3;
        public static final double ARM_EXTEND_KS = 0.1;
        public static final double ARM_EXTEND_KV = 1.31/0.15;//12 /*v*//(5676 /*rpm */ * EXTEND_METERS_PER_DRUM_ROTATION * EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION / 60);
        /* PIVOT */
        public static final double MIN_ARM_ANGLE = 5.86 - 2*Math.PI;
        public static final double MAX_ARM_ANGLE = Math.PI + 0.42;
        public static final double ARM_ROTATIONS_PER_MOTOR_ROTATION = 1.0/225.0; //(1.0/25.0) * (16.0/60.0);

        public static final double PIVOT_ENCODER_OFFSET = 0.423 * Math.PI * 2.0;

        public static final double ARM_MASS_KILOS = Units.lbsToKilograms(21.1);

        public static final double ARM_MOI_SHRUNK = 0.7975;
        
        public static final int PIVOT_MOTOR_ID = 22;
        public static final int PIVOT_FOLLOWER_MOTOR_ID = 23;
        public static final double PIVOT_KS = 0.11 * 0.6 ;
        public static final double ARM_PIVOT_KG_MIN_EXTEND = 0.1;//1.414 * (ARM_ROTATIONS_PER_MOTOR_ROTATION * 400) / 2 / Math.cos(Units.degreesToRadians(10.5));
        public static final double ARM_PIVOT_KG_MAX_EXTEND = 1;//0.13 / Math.cos(0.707);//2.872 * (ARM_ROTATIONS_PER_MOTOR_ROTATION * 400) / 2 / Math.cos(Units.degreesToRadians(10.5));

        public static final double PIVOT_MAX_VELOCITY = 2.38; // rad/s
        public static final double PIVOT_MAX_ACCEL = 5; //rad/s/s

        /* WRIST/HAND */

        // Wrist angle is relative to the extension of the telescope.
        // I.e. 0 is straight out from the pivot.
        // positive angles match pivot (up and over, with 0 being straight out the robot front)
        public static final int WRIST_MOTOR_ID = 25;
        public static final double WRIST_ROTATIONS_PER_MOTOR_ROTATION = 1.0/300.0;
        public static final double WRIST_ENCODER_OFFSET = 0.0972610;//5.382308 - Math.PI;//0.995;
        public static final double HAND_LENGTH = Units.inchesToMeters(8);
        public static final double HAND_MASS_KILOS = Units.lbsToKilograms(5);
      
        public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-60);
        public static final double WRIST_MAX_ANGLE = 1.5;
        public static final double WRIST_KG = 5.744 * 1.6 * 5 / (300.0);

        public static final ArmPosition SCORE_HIGH_CONE_POSITION = new ArmPosition(
            0.658,
            1.429,
            -Units.degreesToRadians(10),
            Units.inchesToMeters(9.4));
        public static final ArmPosition SCORE_HIGH_CUBE_POSITION = new ArmPosition(
        0.658 - Units.degreesToRadians(10),
        1.2,
        (Math.PI/2)-1,
        Units.inchesToMeters(9.4));
        public static final ArmPosition SCORE_MID_CONE_POSITION = new ArmPosition(
            0.74,
            1.082,
            -0.74,
            Units.inchesToMeters(9.4));
        public static final ArmPosition SCORE_MID_CUBE_POSITION = new ArmPosition(
            0.658 - Units.degreesToRadians(15),
            0.75,
            (Math.PI/2)-1,
            Units.inchesToMeters(9.4));
        public static final ArmPosition RETRACTED_SCORE_CONE_POSITION = new ArmPosition(
            0.658,
            0.628,
            0,
            Units.inchesToMeters(9.4));
        public static final ArmPosition STOW_POSITION = new ArmPosition(
            Units.degreesToRadians(66),
            MIN_ARM_LENGTH + Units.inchesToMeters(1),
            WRIST_MAX_ANGLE - Units.degreesToRadians(5),
            Units.inchesToMeters(9.4));
        public static final ArmPosition GROUND_CUBE_INTAKE_POSITION = new ArmPosition(
            -0.41,
            0.611,
            0.56,
            Units.inchesToMeters(9.4));
        public static final ArmPosition GROUND_CONE_INTAKE_POSITION = new ArmPosition(
            -0.42,
            0.611,
            WRIST_MAX_ANGLE - Units.degreesToRadians(8),
            Units.inchesToMeters(9.4));
        public static final ArmPosition RAMP_CONE_INTAKE_POSITION = new ArmPosition(
            2.682,
            0.509,
            Units.degreesToRadians(71.127),
            Units.inchesToMeters(9.4)
        );

        public static final ArmPosition RAMP_CUBE_INTAKE_POSITION = new ArmPosition(
            2.768,
            0.516,
            Units.degreesToRadians(37.755),
            Units.inchesToMeters(9.4)
        );
        public static final ArmPosition RAMP_CUBE_INTAKE_POSITION_FRONT = new ArmPosition(
            0.3 - Units.degreesToRadians(3),
            MIN_ARM_LENGTH + Units.inchesToMeters(0.125),
            WRIST_MAX_ANGLE - Units.degreesToRadians(1),
            Units.inchesToMeters(9.4)
        );

        public static final ArmPosition PLATFORM_CONE_INTAKE_POSITION = new ArmPosition(
            1.130,
            0.764,
            MathUtil.angleModulus(Units.degreesToRadians(339.446)),
            Units.inchesToMeters(9.4)
        );
        public static final ArmPosition PLATFORM_CUBE_INTAKE_POSITION = new ArmPosition(
            1.130,
            0.764,
            MathUtil.angleModulus(Units.degreesToRadians(339.446)),
            Units.inchesToMeters(9.4)
        );
        public static final ArmPosition OVERTOP_CONE_INTAKE_POSITION = new ArmPosition(
            3.5,
            0.628,
            1.400,
            Units.inchesToMeters(9.4)
        );
        public static final ArmPosition OVERTOP_CUBE_INTAKE_POSITION = new ArmPosition(
            3.3,
            0.628,
            1.5,
            Units.inchesToMeters(9.4)
        );
        // public static final ArmPosition HYBRID_NODE_OUTTAKE_POSITION = new ArmPosition(
        //     Units.degreesToRadians(66),
        //     MIN_ARM_LENGTH + Units.inchesToMeters(0.125),
        //     Units.degreesToRadians(-15),
        //     Units.inchesToMeters(9.4));

        public static final ArmPosition SCORE_HYBRID_POSITION = new ArmPosition(
            Units.degreesToRadians(66),
            MIN_ARM_LENGTH + Units.inchesToMeters(0.125),
            Units.degreesToRadians(-50),
            Units.inchesToMeters(9.4));

        
    }


    public static final class IntakeConstants {
        public static final int INTAKE_CAN_ID = 30; 
        public static final double INTAKE_VOLTAGE = 6.995 / 3.0;
        public static final int INTAKE_EXTEND = 0;
        public static final int INTAKE_RETRACT= 1;
        public static final int INTAKE_FOLLOWER_CAN_ID = 31 ;
        public static final int INTAKE_TOF_CAN_ID = 32;
        public static final double INTAKE_CENTERED_CONE_DISTANCE = 130.0;
    }

    public static final class LightConstants {
        public static final double LED_BOUNCE_GREEN = 0.77;
        public static final double LED_SINELON_OCEAN = -0.75;
        public static final double LED_LIGHT_CHASE_RED = -0.31;
        public static final double LED_PARTY_MODE = -0.43;
        public static final double LED_SOLID_YELLOW = 0.69;
        public static final double LED_SOLID_VIOLET = 0.91;
        public static final double LED_STROBE_GOLD = -0.07;

        public static final int PWM_PORT_LED = 0;
    }
}
