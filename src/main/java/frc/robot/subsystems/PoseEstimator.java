// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
/*
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
*/

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PoseEstimator extends SubsystemBase {

  // Pose estimator standard deviations
  private static final edu.wpi.first.math.Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05,
      Units.degreesToRadians(0.05));
  private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5,
      Units.degreesToRadians(5));

  // Pose estimator, field image
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d;

  // Drive subsystem
  private Drive drive;

  // Limelight
  private NetworkTable limelightNetworkTable;
  private double cameraLatency;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private PhotonCamera intake = new PhotonCamera("USB_Camera");

  /*
   * // Photon cameras
   * private PhotonCamera leftCamera = new PhotonCamera("left-camera");
   * //private PhotonCamera rightCamera = new PhotonCamera("right-camera");
   * 
   * // Photon pose estimators for left & right cameras
   * private PhotonPoseEstimator leftPoseEstimator = new
   * PhotonPoseEstimator(aprilTagFieldLayout,
   * PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera,
   * Constants.Vision.cameraLeftToCenter);
   * private Pose3d leftPose;
   * /*private PhotonPoseEstimator rightPoseEstimator = new
   * PhotonPoseEstimator(aprilTagFieldLayout,
   * PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera,
   * Constants.Vision.cameraRightToCenter);
   * private Pose3d rightPose;
   * 
   */

  public PoseEstimator(Drive drive) {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    this.drive = drive;

    field2d = new Field2d();

    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

     poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        this.drive.getHeading(),
        this.drive.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    visionTab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    visionTab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(Constants.Vision.limelightName);

  }

  @Override
  public void periodic() {

    Pose3d bestPose = getDesiredPose();

    if (bestPose != null)
      poseEstimator.addVisionMeasurement(bestPose.toPose2d(), Timer.getFPGATimestamp() - cameraLatency);

    poseEstimator.update(
        drive.getHeading(),
        drive.getModulePositions());

    field2d.setRobotPose(getCurrentPose());

    SmartDashboard.putNumber("Rotation to speaker", getRotationToSpeaker(RobotContainer.isRed()));
    SmartDashboard.putNumber("X to speaker", getXDistanceToRedSpeaker());
    SmartDashboard.putNumber("Y to speaker", getYDistanceToRedSpeaker());
    SmartDashboard.putNumber("Gyro Heading", drive.getHeading().getDegrees());
    
    SmartDashboard.putNumber("Elevation to speaker", getElevationAngle(RobotContainer.isRed()));

    SmartDashboard.putBoolean("Red", RobotContainer.isRed());

  }

  // Returns pose to be used in Shuffleboard
  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  // Returns pose estimator pose
  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Resets pose estimator pose
  public void resetPose() {
    poseEstimator.resetPosition(drive.getHeading(), drive.getModulePositions(), getCurrentPose());
  }

  // Returns the best vision pose, for updating pose estimator
  private Pose3d getDesiredPose() {

    // If limelight tag has enough area, favor it over side cameras
    if (getLimelightArea() > Constants.Vision.minimumLimelightArea) {
      double[] limelightBotPose = getBotPoseBlue();

      cameraLatency = limelightBotPose[6] / 1000.0;

      return new Pose3d(limelightBotPose[0], limelightBotPose[1], limelightBotPose[2],
          new Rotation3d(Units.degreesToRadians(limelightBotPose[3]), Units.degreesToRadians(limelightBotPose[4]),
              Units.degreesToRadians(limelightBotPose[5])));
    }
    return null;
    /*
     * 
     * var leftResult = leftCamera.getLatestResult();
     * var rightResult = rightCamera.getLatestResult();
     * 
     * // If side cameras cannot see any tags return null
     * if (!leftResult.hasTargets() && !rightResult.hasTargets()) {
     * return null;
     * }
     * 
     * // Compare left & right camera ambiguity & return better pose
     * if (leftResult.hasTargets()
     * && leftResult.getBestTarget().getPoseAmbiguity() >
     * rightResult.getBestTarget().getPoseAmbiguity()) {
     * // Return left camera pose
     * 
     * leftPoseEstimator.update().ifPresent((pose) -> {
     * cameraLatency = pose.timestampSeconds / 1000.0;
     * leftPose = pose.estimatedPose;
     * });
     * return leftPose;
     * }
     * 
     * // Return right camera pose
     * rightPoseEstimator.update().ifPresent((pose) -> {
     * cameraLatency = pose.timestampSeconds / 1000.0;
     * rightPose = pose.estimatedPose;
     * });
     * return rightPose;
     */
  }

  // Returns limelight tag area
  private double getLimelightArea() {
    return limelightNetworkTable.getEntry("ta").getDouble(0);
  }

  // Returns limelight pose relative to blue origin
  private double[] getBotPoseBlue() {
    return limelightNetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }

  public void setLimelightLight(int mode) {
    limelightNetworkTable.getEntry("ledMode").setNumber(mode);
  }

  private double getDistanceToRedSpeaker() {
    return Math.sqrt(Math.pow(getCurrentPose().getX() - Constants.Vision.speakerRedTranslation.getX(), 2)
        + Math.pow(getCurrentPose().getY() - Constants.Vision.speakerRedTranslation.getY(), 2));
  }

  private double getDistanceToBlueSpeaker() {
    return Math.sqrt(Math.pow(getCurrentPose().getX() - Constants.Vision.speakerBlueTranslation.getX(), 2)
        + Math.pow(getCurrentPose().getY() - Constants.Vision.speakerBlueTranslation.getY(), 2));
  }

  public double getDistanceToSpeaker() {
    return RobotContainer.isRed() ? getDistanceToRedSpeaker() : getDistanceToBlueSpeaker();
  }

  private double getXDistanceToRedSpeaker() {
    return Math.abs(getCurrentPose().getX() - Constants.Vision.speakerRedTranslation.getX());
  }

  private double getYDistanceToRedSpeaker() {
    return getCurrentPose().getY() - Constants.Vision.speakerRedTranslation.getY();
  }

  private double getXDistanceToBlueSpeaker() {
    return Math.abs(getCurrentPose().getX() - Constants.Vision.speakerBlueTranslation.getX());
  }

  private double getYDistanceToBlueSpeaker() {
    return getCurrentPose().getY() - Constants.Vision.speakerBlueTranslation.getY();
  }

  // Returns in degrees
  public double getElevationAngle(boolean red) {
    return Units.radiansToDegrees(
        Math.atan(Constants.Vision.speakerHeight / (red ? getDistanceToRedSpeaker() : getDistanceToBlueSpeaker())));
  }

  public double getRotationToSpeaker(boolean red) {
    if (getCurrentPose().getY() > Constants.Vision.speakerRedTranslation.getY()) {
      return Math.toDegrees((Math.atan((red ? getXDistanceToRedSpeaker() : getXDistanceToBlueSpeaker())
          / (red ? getYDistanceToRedSpeaker() : getYDistanceToBlueSpeaker())))) - 90;
    }
    return 90 + Math.toDegrees((Math.atan((red ? getXDistanceToRedSpeaker() : getXDistanceToBlueSpeaker())
        / (red ? getYDistanceToRedSpeaker() : getYDistanceToBlueSpeaker()))));
  }
}