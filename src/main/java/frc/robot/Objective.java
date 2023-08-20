// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.ArmS.ArmPosition;

/** Add your docs here. */
public class Objective {
    Pose2d frontPose;
    Pose2d backPose;
    ArmPosition frontArm;
    ArmPosition backArm;
    boolean cubeMode;

    /** Creates an Objective where the back pose and position are simply mirrored versions of the front. */
    public Objective(Pose2d frontPose, ArmPosition frontArm, boolean cubeMode) {
        this.cubeMode = cubeMode;
        this.frontPose = frontPose;
        this.backPose = frontPose.transformBy(new Transform2d(new Translation2d(), new Rotation2d(Math.PI)));
        this.frontArm = frontArm;
        this.backArm = new ArmPosition(Math.PI - frontArm.pivotRadians, frontArm.armLength, -frontArm.pivotRadians);
    }

    public Objective(Pose2d frontPose, Pose2d backPose, ArmPosition frontArm, boolean cubeMode) {
        this.cubeMode = cubeMode;
        this.frontPose = frontPose;
        this.backPose = frontPose.transformBy(new Transform2d(new Translation2d(), new Rotation2d(Math.PI)));
        this.frontArm = frontArm;
        this.backArm = new ArmPosition(Math.PI - frontArm.pivotRadians, frontArm.armLength, -frontArm.pivotRadians);
    }
}
