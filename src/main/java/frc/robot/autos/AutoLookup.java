// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.DetectIntake;
import frc.robot.commands.ShooterCommands.FeedToShooter;
import frc.robot.commands.ShooterCommands.ShooterElevation;

/** Add your docs here. */
public class AutoLookup {

    // static PathPlannerPath CenterOut;
    static PathPlannerPath path2;

    public AutoLookup() {
        // CenterOut = PathPlannerPath.fromPathFile("Middle 2p");
        path2 = PathPlannerPath.fromPathFile("Left Line");
    }

    public static Command getAuto(String name) {
        switch (name) {
            case "Middle 2p":
                return new SequentialCommandGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("1st step")),

                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(65)),
                        // new ShooterElevation(RobotContainer.getShooter()).withTimeout(0.3),
                        // new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(65)),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.7, 0.7)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                        new ParallelCommandGroup(

                                // new GoAndRotate(RobotContainer.getDrive(), 0.75, 0).withTimeout(2.75),

                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-1")),

                                RobotContainer.getIntakeCommand()

                        ),

                        new WaitCommand(0.2),
                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(45)),

                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))

                // AutoBuilder.followPath(path1)

                );

            case "Middle 3p-podium":
                return new SequentialCommandGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("1st step")),

                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(65)),
                        // new ShooterElevation(RobotContainer.getShooter()).withTimeout(0.3),
                        // new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(65)),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.4, 0.4)),
                        new WaitCommand(0.6),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                        new WaitCommand(0.6),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                        new ParallelCommandGroup(

                                // new GoAndRotate(RobotContainer.getDrive(), 0.75, 0).withTimeout(2.75),

                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-1")),

                                RobotContainer.getIntakeCommand()),

                        new WaitCommand(0.2),
                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(45)),

                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-2")),
                                RobotContainer.getIntakeCommand()

                        ),

                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("temp path")),

                        new WaitCommand(0.2),
                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(45)),

                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))

                );

            case "Only Shoot":

                return new SequentialCommandGroup(
                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(65)),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.6, 0.6)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))

                );
            // return AutoBuilder.followPath(path2);

            case "Ampside 4p":
                return new SequentialCommandGroup(
                        // shoot1
                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(55)),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.6, 0.6)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                        new WaitCommand(1),
                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),

                        // path to note 1
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Ampside 4p-1")),
                                new SequentialCommandGroup(

                                ))

                );
            case "TEST":
                return new SequentialCommandGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("TEST")));
            default:
                return null;
        }
    }
}
