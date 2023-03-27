package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.ArmConstants;

public class ArmConstraints {
    private InterpolatingTreeMap<Double, Double> functionExtended = new InterpolatingTreeMap<>();
    private InterpolatingTreeMap<Double, Double> functionRetracted = new InterpolatingTreeMap<>();
    private InterpolatingTreeMap<Double, Double> functionCurrent = new InterpolatingTreeMap<>();
    private Field2d VISUALIZER;
    private double wristAngle = 0;
    private double handLength = Units.inchesToMeters(16);

    List<Function<Double, Double>> constraints = List.of(
        (angle)->ArmConstants.MAX_ARM_LENGTH,
        (angle)-> (Units.inchesToMeters(12.5 + 48) - (handLength * Math.cos(/*wristAngle + */angle))) / Math.cos(angle),
        (angle)-> -(Units.inchesToMeters(12.5 + 48) + (handLength * Math.cos(/*wristAngle + */angle))) / Math.cos(angle),
        (angle)-> (
            Units.inchesToMeters(78) - (
                handLength * Math.sin(wristAngle + angle)
            ) - ArmConstants.ARM_PIVOT_TRANSLATION.getY()
        ) / Math.sin(angle),
        (angle)-> (
            
                -(handLength * Math.sin(wristAngle + angle)
            ) - ArmConstants.ARM_PIVOT_TRANSLATION.getY()
        ) / Math.sin(angle),
        (angle)-> (
            
         - ArmConstants.ARM_PIVOT_TRANSLATION.getY()
        ) / Math.sin(angle)

    );
    // private double wristXMax = Units.inchesToMeters(12.5 + 48) - handLength;
    // private double wristYMax = Units.inchesToMeters(78) - handLength - ArmConstants.ARM_PIVOT_TRANSLATION.getY();
    // List<Function<Double, Double>> constraints = List.of(
    //     (angle)->ArmConstants.MAX_ARM_LENGTH,
    //     (angle)-> wristXMax / Math.abs(Math.cos(angle)),
    //     (angle)-> wristYMax / Math.sin(angle)
    // );

    public ArmConstraints(Field2d VISUALIZER) {
        this.VISUALIZER = VISUALIZER;
        List<Pose2d> vizPosesExtended = new LinkedList<>();
        List<Pose2d> vizPosesRetracted = new LinkedList<>();
        functionExtended = generateConstraints(Units.inchesToMeters(16));
        functionRetracted = generateConstraints(Units.inchesToMeters(9.4));
        for (double i = ArmConstants.MIN_ARM_ANGLE; i < ArmConstants.MAX_ARM_ANGLE; i+= 0.01) {
            vizPosesExtended.add(new Pose2d(i, functionExtended.get(i), new Rotation2d()));
            vizPosesRetracted.add(new Pose2d(i, functionRetracted.get(i), new Rotation2d()));
        }
        VISUALIZER.getObject("maximumExtended").setPoses(vizPosesExtended);
        VISUALIZER.getObject("maximumRetracted").setPoses(vizPosesRetracted);
    }

    private InterpolatingTreeMap<Double, Double> generateConstraints(double handLength) {
        var function = new InterpolatingTreeMap<Double, Double>();
        List<Function<Double, Double>> constraints = List.of(
            (angle)->ArmConstants.MAX_ARM_LENGTH,
            (angle)-> getWristXMax(handLength) / Math.abs(Math.cos(angle)),
            (angle)-> getWristYMax(handLength) / Math.sin(angle)
        );
        for (double i = ArmConstants.MIN_ARM_ANGLE; i < ArmConstants.MAX_ARM_ANGLE; i+= 0.01) {
            var length = maxLength(i, constraints);
            function.put(i, length);
        }
        return function;
    }

    public void update(double handLength, double wristAngle) {
        this.wristAngle = wristAngle;
        this.handLength = handLength;
        List<Pose2d> vizPosesCurrent = new LinkedList<>();
        

        functionCurrent.clear();
        for (double i = ArmConstants.MIN_ARM_ANGLE; i < ArmConstants.MAX_ARM_ANGLE; i+= 0.01) {
            var length = maxLength(i, constraints);
            functionCurrent.put(i, length);
            vizPosesCurrent.add(new Pose2d(i, functionCurrent.get(i), new Rotation2d()));
        }

        for (double i = ArmConstants.MAX_ARM_ANGLE; i > ArmConstants.MIN_ARM_ANGLE; i-= 0.01) {
            vizPosesCurrent.add(new Pose2d(i, getMinLength(i), new Rotation2d()));
        }
        VISUALIZER.getObject("maximumCurrent").setPoses(vizPosesCurrent);

        //return function;
    }
    private double getWristXMax(double handLength) {
        return Units.inchesToMeters(12.5 + 48) - handLength;
    }

    private double getWristYMax(double handLength) {
        return Units.inchesToMeters(78) - handLength - ArmConstants.ARM_PIVOT_TRANSLATION.getY();
    }

    public double maxLength(double angle, List<Function<Double, Double>> constraints) {
        double maxLength = Double.MAX_VALUE;
        for (var constraint : constraints) {
            double constrainedLength = constraint.apply(angle); 
            if (constrainedLength < maxLength && constrainedLength > 0) {
                maxLength = constrainedLength;
            }
        }
        return Math.max(maxLength, getMinLength(angle));
    }

    public double getMinLength(double angle) {
        if (angle < 0 || angle > Math.PI) {
            return 0.610;
        }
        return ArmConstants.MIN_ARM_LENGTH + Units.inchesToMeters(0.125);
    }

    public double constrainLength(double length, double angle) {
        return MathUtil.clamp(length, getMinLength(angle), maxLength(angle, constraints));
    }
}
