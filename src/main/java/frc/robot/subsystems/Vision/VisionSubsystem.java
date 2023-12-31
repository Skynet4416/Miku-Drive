// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera mCamera;
  private AprilTagFieldLayout mLayout;
  private final Transform3d mMiddleToCamera;
  private PhotonPoseEstimator mPoseEstimator;
  

  public VisionSubsystem() {
    // ! CHANGE CAMERA NAME !!!!!!!!!!!
    this.mCamera = new PhotonCamera("nof");
    this.mLayout = null;
    this.mMiddleToCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    this.mPoseEstimator = new PhotonPoseEstimator(mLayout, PoseStrategy.AVERAGE_BEST_TARGETS, mMiddleToCamera);
    try {
      // ! CHANGE TO ACTUAL FIELD - YOTAM HAS LEARNT FROM THIS MISTAKE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      this.mLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException exception) {
      System.out.println(exception);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      mPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return mPoseEstimator.update();
  }

  public boolean hasTarget() {
    return mCamera.getLatestResult().hasTargets();
  }

  public PhotonTrackedTarget getTarget() {
    return mCamera.getLatestResult().getBestTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}