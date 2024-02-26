// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class AlertCommand extends Command {

  private CommandXboxController controller;
  boolean useRumble = false;
  boolean useLimelight = false;
  
  public AlertCommand(CommandXboxController controller, boolean useRumble, boolean useLimelight) {
    this.controller = controller;
    this.useRumble = useRumble;
    this.useLimelight = useLimelight;
  }

  @Override
  public void initialize() {
    if(useRumble)
      controller.getHID().setRumble(RumbleType.kBothRumble, 0.4);

    if(useLimelight)
      NetworkTableInstance.getDefault().getTable(Constants.Vision.limelightName).getEntry("ledMode").setNumber(2);

    // 1: Off, 2: Blink, 3: Solid
  }
  
  @Override
  public void execute() {
  }
  
  @Override
  public void end(boolean interrupted) {
    controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    NetworkTableInstance.getDefault().getTable(Constants.Vision.limelightName).getEntry("ledMode").setNumber(1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
