// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX leftClimberMotor;
  private TalonFX rightClimberMotor;

  public Climber() {
    leftClimberMotor = new TalonFX(Constants.Climber.leftID);
    rightClimberMotor = new TalonFX(Constants.Climber.rightID);
  }

  @Override
  public void periodic() {
  }

  public void setLeftClimber(double speed) {
    leftClimberMotor.set(-speed);
  }

  public void setRightClimber(double speed) {
    rightClimberMotor.set(speed);
  }
}
