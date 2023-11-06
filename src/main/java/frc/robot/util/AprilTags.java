package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTags {
    public static Matrix<N3, N1> calculateVisionUncertainty(double poseX, Rotation2d heading, Rotation2d cameraYaw, String cameraName) {
        double maximumUncertainty = 3;
        double minimumUncertainty = 0.1;
        double a = 6;
        double b = -1.3;
        Rotation2d cameraWorldYaw = cameraYaw.rotateBy(heading);
        boolean isCameraFacingFieldSideTags;
        boolean facingRedAlliance;
        double distanceFromTagSide;

        if(-90 < cameraWorldYaw.getDegrees() && cameraWorldYaw.getDegrees() < 90) {
            // camera facing towards red alliance
            facingRedAlliance = true;
            isCameraFacingFieldSideTags = poseX > 16.5 / 2;
        } else {
            // camera facing towards blue alliance
            facingRedAlliance = false;
            isCameraFacingFieldSideTags = poseX < 16.5 / 2;
        }

        if(isCameraFacingFieldSideTags) {
            // uncertainty low
            if(facingRedAlliance) {
                distanceFromTagSide = 16.5 - poseX;
            } else {
                distanceFromTagSide = poseX;
            }
        } else {
            // uncertainty high
            if(!facingRedAlliance) {
                distanceFromTagSide = poseX;
            } else {
                distanceFromTagSide = 16.5 - poseX;
            }
        }
        double positionUncertainty = ((maximumUncertainty-minimumUncertainty)/(1+Math.pow(Math.E, (a+(b*distanceFromTagSide)))))+minimumUncertainty;

        SmartDashboard.putNumber("Debug/Cameras/"+cameraName+"/Distance From Tag Side", distanceFromTagSide);
        SmartDashboard.putNumber("Debug/Cameras/"+cameraName+"/Vision Uncertainty", positionUncertainty);
        SmartDashboard.putBoolean("Debug/Cameras/"+cameraName+"/Facing Red Alliance", facingRedAlliance);

        return VecBuilder.fill(positionUncertainty,positionUncertainty,10000);
    }
}