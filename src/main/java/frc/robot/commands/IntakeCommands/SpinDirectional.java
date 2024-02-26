// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SpinDirectional extends Command {
  private Intake intake;
  private double speed;

  public SpinDirectional(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.setDirectional(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setDirectional(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
