// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeSequence extends ParallelCommandGroup {
  private Intake intake;

  public IntakeSequence() {
    intake = RobotContainer.getIntake();

    addCommands(
        // Set rollers
        new InstantCommand(() -> intake.setRollers(0.5)),

        // Wait for TOF to activate
        new DetectIntake(intake),

        // Turn off rollers
        new InstantCommand(() -> intake.setRollers(0)));
  }
}
