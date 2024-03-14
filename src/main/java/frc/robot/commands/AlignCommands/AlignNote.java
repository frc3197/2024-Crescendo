// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlignCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

public class AlignNote extends Command {
  /** Creates a new AlignNote. */
  private Vision vision;
  private Drive drive;
  private Intake intake;

  private double maxRotSpeed = 3;

  public AlignNote(Vision vision, Drive drive, Intake intake) {
    this.vision = vision;
    this.drive = drive;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Not rot", Vision.getBestTarget()[0]);
    drive.drive(new Translation2d(RobotContainer.isRed() ? -1 : -1 * MathUtil.clamp(Vision.getBestTarget()[1]/2.5, 1.25, 3), 0), -MathUtil.clamp(Vision.getBestTarget()[0]/14, -maxRotSpeed, maxRotSpeed), false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new Translation2d(0, 0), 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getRange() < Constants.Intake.TOFRange;
  }
}
