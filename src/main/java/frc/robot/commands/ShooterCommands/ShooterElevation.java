// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterElevation extends Command {

  private Shooter shooter;

  private PIDController elevationPID;

  private double maxSpeed = 0.65;

  public ShooterElevation(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);

    shooter.setTargetAngle(shooter.getShooterAngle());

    this.elevationPID = new PIDController(
        Constants.Shooter.elevationP,
        Constants.Shooter.elevationI,
        Constants.Shooter.elevationD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = elevationPID
        .calculate(MathUtil.applyDeadband(shooter.getShooterAngle() - shooter.getTargetAngle(), 0.01)) / 16.0;

    speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);

    shooter.setElevationMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setElevationMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
