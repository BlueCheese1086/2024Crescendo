// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem
 */
public class Vision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("kitcam");

  /**
   * Creates a new Vision subsystem.
   */
  public Vision() {
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      System.out.println("Has targets.");
      PhotonTrackedTarget target = result.getBestTarget();
      System.out.println(target.getFiducialId());
    }
  }
}
