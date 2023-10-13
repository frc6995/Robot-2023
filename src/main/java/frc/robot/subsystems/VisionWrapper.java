// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.Config;
import frc.robot.Constants.PoseEstimator;
import frc.robot.Constants.Vision;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.util.AprilTags;
import frc.robot.util.CSV;


public class VisionWrapper {
  /** If shuffleboard should be used--important for unit testing. */
  private static boolean useShuffleboard = false;

  record CameraEstimator(PhotonCamera camera, PhotonPoseEstimator estimator) {}

  private final List<CameraEstimator> cameraEstimators = new ArrayList<>();

  private AprilTagFieldLayout fieldLayout;

  private double lastDetection = 0;

  private Supplier<Rotation2d> headingSupplier = ()->new Rotation2d();

  /** Creates a new VisionSubsystem. */
  public VisionWrapper(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = headingSupplier;
    // loading the 2023 field arrangement
    fieldLayout = VisionConstants.TAG_FIELD_LAYOUT;

    for (Vision.VisionSource visionSource : Vision.VISION_SOURCES) {
      var camera = new PhotonCamera(visionSource.name());
      var estimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
              camera,
              visionSource.robotToCamera());
      estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
      cameraEstimators.add(new CameraEstimator(camera, estimator));
    }

    // if (useShuffleboard)
    //   cameraStatusList.addString(
    //       "time since apriltag detection",
    //       () -> String.format("%3.0f seconds", Timer.getFPGATimestamp() - lastDetection));

    var thread =
        new Thread(
            () -> {
              if (fieldLayout == null) return;
              while (!Thread.currentThread().isInterrupted()) {
                this.findVisionMeasurements();
                try {
                  Thread.sleep(Vision.THREAD_SLEEP_DURATION_MS);
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                }
              }
            });
    thread.setDaemon(true);
    thread.start();
  }

  record MeasurementRow(
      double realX,
      double realY,
      int tags,
      double avgDistance,
      double ambiguity,
      double estX,
      double estY,
      double estTheta) {}

  private final CSV<MeasurementRow> measurementCSV =
      Config.WRITE_APRILTAG_DATA
          ? new CSV<>(
              Config.APRILTAG_DATA_PATH,
              List.of(
                  CSV.column("realX", MeasurementRow::realX),
                  CSV.column("realY", MeasurementRow::realY),
                  CSV.column("tags", MeasurementRow::tags),
                  CSV.column("avgDistance", MeasurementRow::avgDistance),
                  CSV.column("ambiguity", MeasurementRow::ambiguity),
                  CSV.column("estX", MeasurementRow::estX),
                  CSV.column("estY", MeasurementRow::estY),
                  CSV.column("estTheta", MeasurementRow::estTheta)))
          : null;

  private void logMeasurement(int tags, double avgDistance, double ambiguity, Pose3d est) {
    if (!Config.WRITE_APRILTAG_DATA) return;

    measurementCSV.write(
        new MeasurementRow(
            Config.REAL_X,
            Config.REAL_Y,
            tags,
            avgDistance,
            ambiguity,
            est.toPose2d().getTranslation().getX(),
            est.toPose2d().getTranslation().getY(),
            est.toPose2d().getRotation().getRadians()));
  }

  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double minimum) {
    private double computeUnitDeviation(double averageDistance) {
      return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
    }
  }

  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    private Matrix<N3, N1> computeDeviation(double averageDistance) {
      return Matrix.mat(Nat.N3(), Nat.N1())
          .fill(
              xParams.computeUnitDeviation(averageDistance),
              yParams.computeUnitDeviation(averageDistance),
              thetaParams.computeUnitDeviation(averageDistance));
    }

    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }
  }

  public static record VisionMeasurement(
      EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {}

  private ConcurrentLinkedQueue<VisionMeasurement> visionMeasurements =
      new ConcurrentLinkedQueue<>();

  private static boolean ignoreFrame(PhotonPipelineResult frame) {
    if (!frame.hasTargets() || frame.getTargets().size() > PoseEstimator.MAX_FRAME_FIDS)
      return true;

    boolean possibleCombination = false;
    List<Integer> ids = frame.targets.stream().map(t -> t.getFiducialId()).toList();
    for (Set<Integer> possibleFIDCombo : PoseEstimator.POSSIBLE_FRAME_FID_COMBOS) {
      possibleCombination = possibleFIDCombo.containsAll(ids);
      if (possibleCombination) break;
    }
    if (!possibleCombination) System.out.println("Ignoring frame with FIDs: " + ids);
    return !possibleCombination;
  }

  public VisionMeasurement drainVisionMeasurement() {
    return visionMeasurements.poll();
  }

  public void findVisionMeasurements() {
    for (CameraEstimator cameraEstimator : cameraEstimators) {
      cameraEstimator.camera().setDriverMode(false);
      PhotonPipelineResult frame = cameraEstimator.camera().getLatestResult();

      // determine if result should be ignored
      if (ignoreFrame(frame)) continue;

      var optEstimation = cameraEstimator.estimator().update(frame);
      if (optEstimation.isEmpty()) continue;
      var estimation = optEstimation.get();

      if (estimation.targetsUsed.size() == 1
          && (estimation.targetsUsed.get(0).getPoseAmbiguity() > PoseEstimator.POSE_AMBIGUITY_CUTOFF
              || estimation.targetsUsed.get(0).getPoseAmbiguity() == -1)) continue;

      // double sumDistance = 0;
      // for (var target : estimation.targetsUsed) {
      //   var t3d = target.getBestCameraToTarget();
      //   sumDistance +=
      //       Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      // }
      // double avgDistance = sumDistance / estimation.targetsUsed.size();

      var deviation =
      AprilTags.calculateVisionUncertainty(
        estimation.estimatedPose.getX(),
        headingSupplier.get(),
        new Rotation2d(cameraEstimator.estimator().getRobotToCameraTransform().getRotation().getZ()),
        cameraEstimator.camera.getName());
          // PoseEstimator.TAG_COUNT_DEVIATION_PARAMS
          //     .get(
          //         MathUtil.clamp(
          //             estimation.targetsUsed.size() - 1,
          //             0,
          //             PoseEstimator.TAG_COUNT_DEVIATION_PARAMS.size() - 1))
          //     .computeDeviation(avgDistance);

      // System.out.println(
      //     String.format(
      //         "with %d tags at smallest distance %f and pose ambiguity factor %f, confidence
      // multiplier %f",
      //         estimation.targetsUsed.size(),
      //         smallestDistance,
      //         poseAmbiguityFactor,
      //         confidenceMultiplier));

      visionMeasurements.add(new VisionMeasurement(estimation, deviation));
    }
  }
}
