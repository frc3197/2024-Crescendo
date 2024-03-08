// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

//import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static PhotonCamera intakeCamera = new PhotonCamera("intake-camera");

  public Vision() {

  }

  @Override
  public void periodic() {
  }

  public boolean hasTarget() {
    var result = intakeCamera.getLatestResult();
    return result.hasTargets();
  }

  public static double[] getBestTarget() {
    var result = intakeCamera.getLatestResult();
    if (result.hasTargets()) {
      var bestTarget = result.getBestTarget();
      return new double[] { bestTarget.getYaw(), bestTarget.getPitch() };
    }
    return new double[] { 0, 0 };
  }
}
