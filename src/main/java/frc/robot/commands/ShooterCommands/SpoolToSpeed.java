// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpoolToSpeed extends Command {

  private Shooter shooter;
  private double bottomSpeed;
  private double topSpeed;

  public SpoolToSpeed(Shooter shooterSub, double bottomSpeed, double topSpeed) {
    this.shooter = shooterSub;
    this.bottomSpeed = bottomSpeed;
    this.topSpeed = topSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spool(bottomSpeed, topSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return shooter.getBottomRollerOutput() > (bottomSpeed - 0.05) && shooter.getTopRollerOutput() > (topSpeed - 0.05);
  }
}
