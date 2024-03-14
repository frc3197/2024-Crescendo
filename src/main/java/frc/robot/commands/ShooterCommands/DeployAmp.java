// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class DeployAmp extends Command {

  private Shooter shooter;
  private double threshold = 0.002;

  private double speed = 0.15;
  private double upSpeed = 0.225;
  private DeflectorDirection direction;



  public DeployAmp(Shooter shooter, DeflectorDirection direction) {
    this.shooter = shooter;
    this.direction = direction;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(direction == DeflectorDirection.DOWN) {
      shooter.setDeflectorMotor(speed);
    } else {
      shooter.setDeflectorMotor(-upSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setDeflectorMotor(0);
  }

  @Override
  public boolean isFinished() {
    if(direction == DeflectorDirection.UP) {
      return shooter.getDeflectorRotation() > Constants.Shooter.deflectorEncoderUp - threshold;
    }
    return shooter.getDeflectorRotation() < threshold + Constants.Shooter.deflectorEncoderDown;
  }
}