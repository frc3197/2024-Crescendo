// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private TalonFX intakRollerMotor;
  private TalonFX directionalMotor;
  private TalonFX deployMotor;

  private TimeOfFlight intakeSensor;

  public Intake() {
    intakRollerMotor = new TalonFX(Constants.Intake.rollerMotorID);
    directionalMotor = new TalonFX(Constants.Intake.directionalMotorID);
    deployMotor = new TalonFX(Constants.Intake.deployMotorID);

    intakeSensor = new TimeOfFlight(Constants.Intake.IntakeTOFID);
    intakeSensor.setRangingMode(RangingMode.Short, 1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Forward", getDeployForwardLimit());
    SmartDashboard.putBoolean("Reverse", getDeployReverseLimit());
    SmartDashboard.putNumber("TOF", getRange());
  }

  public void setRollers(double speed) {
    intakRollerMotor.set(speed);
  }

  public void setDirectional(double speed) {
    directionalMotor.set(speed);
  }

  public void setDeploy(double speed) {
    deployMotor.set(speed);
  }

  public double getRange() {
    return intakeSensor.getRange();
  }

  public boolean getDeployForwardLimit() {
    return deployMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getDeployReverseLimit() {
    return deployMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }
}
