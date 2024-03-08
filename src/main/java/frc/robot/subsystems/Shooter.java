// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

  private TalonFX bottomRoller;
  private TalonFX topRoller;
  private TalonFX feedMotor;
  private TalonFX deflectorMotor;

  private CANSparkMax elevationMotor;

  private DutyCycleEncoder elevationEncoder;
  private DutyCycleEncoder deflectorEncoder;

  private boolean isRed = RobotContainer.isRed();

  private TimeOfFlight shooterSensor;

  private double targetAngle;

  private PoseEstimator poseEstimator;

  private PIDController shootSpeedPID;

  Drive drive;

  public Shooter(PoseEstimator poseEstimator, Drive drive) {
    this.drive = drive;

    this.bottomRoller = new TalonFX(Constants.Shooter.bottomRollerID);
    this.topRoller = new TalonFX(Constants.Shooter.topRollerID);
    this.feedMotor = new TalonFX(Constants.Shooter.feedMotorID);
    this.deflectorMotor = new TalonFX(Constants.Shooter.deflectorMotorID);

    this.elevationEncoder = new DutyCycleEncoder(0);
    this.deflectorEncoder = new DutyCycleEncoder(1);

    this.elevationMotor = new CANSparkMax(Constants.Shooter.elevationID, MotorType.kBrushless);

    shooterSensor = new TimeOfFlight(Constants.Shooter.ShooterTOFID);

    elevationMotor.setInverted(true);
    elevationMotor.burnFlash();

    this.poseEstimator = poseEstimator;

    shootSpeedPID = new PIDController(1.2, 0, 0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Shooter TOF", getRange());
    SmartDashboard.putNumber("Shooter Encoder", elevationEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Deflector Encoder", deflectorEncoder.getAbsolutePosition());

    SmartDashboard.putNumber("Shooter Elevation Angle", getShooterAngle());
    SmartDashboard.putNumber("Shooter Elevation Target", targetAngle);

    if (RobotContainer.getSpeakerElevationButton()) {
      // targetAngle = poseEstimator.getElevationAngle(RobotContainer.isRed()) + 6 +
      // (poseEstimator.getDistanceToSpeaker()/3.5);
      targetAngle = poseEstimator.getElevationAngle(RobotContainer.isRed()) + 0.44*(poseEstimator.getDistanceToSpeaker()) + (drive.getVelocityX()*1.0);
    }
  }

  public void spool(double bottomSpeed, double topSpeed) {
    bottomRoller.set(-shootSpeedPID.calculate(-bottomSpeed));
    //bottomRoller.set(-bottomSpeed);
    topRoller.set(-shootSpeedPID.calculate(topSpeed));
    //topRoller.set(topSpeed);
  }

  public void setFeedMotor(double speed) {
    feedMotor.set(speed);
  }

  public double getSensorSigma() {
    return shooterSensor.getRangeSigma();
  }

  public double getBottomRollerOutput() {
    return bottomRoller.get();
  }

  public double getTopRollerOutput() {
    return topRoller.get();
  }

  public void setElevationMotor(double value) {
    /*if (elevationEncoder.getAbsolutePosition() > Constants.Shooter.maxEncoder && value > 0) {
      return;
    }

    if (elevationEncoder.getAbsolutePosition() < Constants.Shooter.minEncoder && value < 0) {
      return;
    }*/

    elevationMotor.set(value);
  }

  public void setTargetAngle(double value) {
    targetAngle = value;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getRange() {
    return shooterSensor.getRange();
  }

  public double getShooterAngle() {
    return (1 - Math.abs(elevationEncoder.getAbsolutePosition() - Constants.Shooter.minEncoder)
        / Math.abs(Constants.Shooter.maxEncoder - Constants.Shooter.minEncoder))
        * (Constants.Shooter.minAngle - Constants.Shooter.maxAngle) + Constants.Shooter.maxAngle;
  }

  public void setDeflectorMotor(double value) {
    deflectorMotor.set(value);
  }

}
