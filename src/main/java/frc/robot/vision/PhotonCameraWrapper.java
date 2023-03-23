
/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.vision.PhotonPoseEstimator;

public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator robotPoseEstimator;

    public PhotonCameraWrapper(String cameraName, Transform3d robotToCam) {
        AprilTagFieldLayout atfl = VisionConstants.TAG_FIELD_LAYOUT;


        // Forward Camera
        photonCamera =
                new PhotonCamera(
                        cameraName); // Change the name of your camera here to whatever it is in the
        // PhotonVision UI.

        // ... Add other cameras here

        // Assemble the list of cameras & mount locations

        robotPoseEstimator =
                new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY, photonCamera, robotToCam);
        robotPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> result = robotPoseEstimator.update();
        if (result.isPresent() && result.get().estimatedPose != null) {
            return new Pair<Pose2d, Double>(
                    result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
