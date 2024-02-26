// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class SpinIntake extends Command {
  private Intake intake;
  private int range;
  private double speed;
  private boolean endable;

  public SpinIntake(Intake intake, double speed, boolean endable) {
    this.intake = intake;
    this.range = Constants.Intake.TOFRange;
    this.speed = speed;
    this.endable = endable;

    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.setRollers(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRollers(0);
  }

  @Override
  public boolean isFinished() {
    return (intake.getRange() <= range && endable) || RobotContainer.getIntakeCancelButton();
  }
}
