// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class DeployIntake extends Command {
  private Intake intake;
  private boolean deploy;

  public DeployIntake(Intake intake, boolean deploy) {
    this.intake = intake;
    this.deploy = deploy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (deploy)
      intake.setDeploy(-0.5);
    else
      intake.setDeploy(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setDeploy(0);
  }  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((deploy && intake.getDeployReverseLimit()) || (!deploy && intake.getDeployForwardLimit())) || RobotContainer.getIntakeCancelButton();
  }
}
