// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class FeedToShooter extends Command {

  Shooter shooter;

  public FeedToShooter(Shooter shooter) {
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shooter.setFeedMotor(-0.25);
  }

  @Override
  public void end(boolean interrupted) {
    //shooter.setFeedMotor(0);
  }

  @Override
  public boolean isFinished() {
    //return false || RobotContainer.getIntakeCancelButton();
    return shooter.getRange() < 125 || RobotContainer.getIntakeCancelButton();
  }
}
