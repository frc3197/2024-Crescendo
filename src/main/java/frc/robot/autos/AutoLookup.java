// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.nio.file.Path;

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
import frc.robot.commands.AlignCommands.AlignNote;
import frc.robot.commands.AlignCommands.GoAndRotate;
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

                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(65)),
                                                // new ShooterElevation(RobotContainer.getShooter()).withTimeout(0.3),
                                                // new InstantCommand(() ->
                                                // RobotContainer.getShooter().setTargetAngle(65)),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.7, 0.7)),
                                                new WaitCommand(1),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                new WaitCommand(1),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                new ParallelCommandGroup(

                                                                // new GoAndRotate(RobotContainer.getDrive(), 0.75,
                                                                // 0).withTimeout(2.75),

                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("Center 3p-1")),

                                                                RobotContainer.getIntakeCommand()

                                                ),

                                                new WaitCommand(0.2),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(45)),

                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                new WaitCommand(1),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                new WaitCommand(1),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))

                                // AutoBuilder.followPath(path1)

                                );

                        case "Middle 3p-podium":
                                return new SequentialCommandGroup(
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("1st step")),

                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(52)),
                                                // new ShooterElevation(RobotContainer.getShooter()).withTimeout(0.3),
                                                // new InstantCommand(() ->
                                                // RobotContainer.getShooter().setTargetAngle(65)),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.5, 0.5)),
                                                new WaitCommand(0.7),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                new WaitCommand(0.4),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                new ParallelCommandGroup(

                                                                // new GoAndRotate(RobotContainer.getDrive(), 0.75,
                                                                // 0).withTimeout(2.75),

                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("Center 3p-1")),

                                                                RobotContainer.getIntakeCommand()),

                                                // new WaitCommand(0.2),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(43)),

                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                new WaitCommand(.8),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                new WaitCommand(.6),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),

                                                new ParallelCommandGroup(
                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("Center 3p-2")),

                                                                new SequentialCommandGroup(
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setTargetAngle(17)),
                                                                                new DeployIntake(RobotContainer
                                                                                                .getIntake(), true),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getIntake()
                                                                                                .setRollers(0.75)),
                                                                                // Wait for TOF to activate
                                                                                new DetectIntake(RobotContainer
                                                                                                .getIntake())

                                                                // RobotContainer.getIntakeCommand()

                                                                )),

                                                new ParallelCommandGroup( // RED = "temp path"
                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("temp path")),

                                                                new SequentialCommandGroup(

                                                                                new WaitCommand(0.2),
                                                                                // Turn off rollers
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getIntake()
                                                                                                .setRollers(0)),

                                                                                new DeployIntake(RobotContainer
                                                                                                .getIntake(), false),

                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getIntake()
                                                                                                .setRollers(0.2)),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getIntake()
                                                                                                .setDirectional(-0.1)),

                                                                                new FeedToShooter(RobotContainer
                                                                                                .getShooter())
                                                                                                .withTimeout(1),

                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getIntake()
                                                                                                .setRollers(0)),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getIntake()
                                                                                                .setDirectional(0)),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(0))

                                                                )

                                                ),
                                                new WaitCommand(0.2),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(45)),

                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                new WaitCommand(.5),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                new WaitCommand(.7),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))

                                );

                        case "Middle 3p-amp":
                                return new SequentialCommandGroup(
                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                new ParallelCommandGroup(
                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("1st step")),

                                                                new InstantCommand(() -> RobotContainer.getShooter()
                                                                                .setTargetAngle(55)),
                                                                // new
                                                                // ShooterElevation(RobotContainer.getShooter()).withTimeout(0.3),
                                                                // new InstantCommand(() ->
                                                                // RobotContainer.getShooter().setTargetAngle(65)),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.2),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .spool(0.5, 0.5)),
                                                                                new WaitCommand(0.7),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(-0.4)),
                                                                                new WaitCommand(0.4),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .spool(0.0, 0.0)),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(0)))),
                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                new ParallelCommandGroup(

                                                                // new GoAndRotate(RobotContainer.getDrive(), 0.75,
                                                                // 0).withTimeout(2.75),

                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("Center 3p-1")),

                                                                RobotContainer.getIntakeCommand()),

                                                // new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                new ParallelCommandGroup(
                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("Center 3p-2.2")),
                                                                RobotContainer.getIntakeCommand().withTimeout(5),

                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.25),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .setAutoTrackSpeaker(
                                                                                                                true)),

                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .spool(0.8, 0.8)),
                                                                                new WaitCommand(1),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(-0.4)),
                                                                                new WaitCommand(.7),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(0)))),
                                                new ParallelCommandGroup(
                                                                new SequentialCommandGroup(
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(-0.5)),
                                                                                new WaitCommand(1.15),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .spool(0.0, 0.0)),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(0))),

                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.75),
                                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                                .fromPathFile("final"))),

                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(1),
                                                                                RobotContainer.getIntakeCommand()))

                                );

                        case "Only Shoot":

                                return new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(52)),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.6, 0.6)),
                                                new WaitCommand(1),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                new WaitCommand(1),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))

                                );
                        // return AutoBuilder.followPath(path2);

                        case "Ampside 4p":
                                return new SequentialCommandGroup(
                                                // shoot1
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(55)),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.6, 0.6)),
                                                new WaitCommand(1),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                new WaitCommand(1),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),

                                                // path to note 1
                                                new ParallelCommandGroup(
                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("Ampside 4p-1")),
                                                                new SequentialCommandGroup(

                                                                ))

                                );

                        case "Middle 4p":
                                return new SequentialCommandGroup(
                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                new ParallelCommandGroup(
                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("1st step")),

                                                                new InstantCommand(() -> RobotContainer.getShooter()
                                                                                .setTargetAngle(52)),
                                                                // new
                                                                // ShooterElevation(RobotContainer.getShooter()).withTimeout(0.3),
                                                                // new InstantCommand(() ->
                                                                // RobotContainer.getShooter().setTargetAngle(65)),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.2),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .spool(0.5, 0.5)),
                                                                                new WaitCommand(0.8),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(-0.5)),
                                                                                new WaitCommand(0.4),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .spool(0.0, 0.0)),
                                                                                new InstantCommand(() -> RobotContainer
                                                                                                .getShooter()
                                                                                                .setFeedMotor(0)))),
                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                new ParallelCommandGroup(

                                                                // new GoAndRotate(RobotContainer.getDrive(), 0.75,
                                                                // 0).withTimeout(2.75),

                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("Center 3p-1")),

                                                                RobotContainer.getIntakeCommand()),
                                                // new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(43)),
                                                new WaitCommand(0.2),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.5, 0.5)),
                                                new WaitCommand(0.7),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.5)),
                                                new WaitCommand(0.4),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                new ParallelCommandGroup(
                                                                AutoBuilder.followPath(PathPlannerPath
                                                                                .fromPathFile("mid 4p-3")),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(3),
                                                                                RobotContainer.getIntakeCommand()
                                                                                                .withTimeout(5)))

                                // AutoBuilder.followPath(PathPlannerPath.fromPathFile("mid 4p-4")),
                                /*
                                 * new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                 * 
                                 * new WaitCommand(0.2),
                                 * new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                 * new WaitCommand(0.7),
                                 * new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                 * new WaitCommand(0.4),
                                 * new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                 * new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                 * new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false))
                                 */
                                );

                        case "2p amp":
                                return new SequentialCommandGroup(

                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setTargetAngle(60)),
                                                new WaitCommand(0.2),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.6, 0.6)),
                                                new WaitCommand(0.8),
                                                new InstantCommand(
                                                                () -> RobotContainer.getShooter().setFeedMotor(-0.5)),
                                                new WaitCommand(0.4),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.0, 0.0)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                new GoAndRotate(RobotContainer.getDrive(), -2, -1.5).withTimeout(3),
                                                new GoAndRotate(RobotContainer.getDrive(), 0, 0).withTimeout(1)

                                // AutoBuilder.followPath(PathPlannerPath.fromPathFile("1.1"))
                                );

                        case "TEST":
                                return new SequentialCommandGroup(
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("TEST red")));

                        case "test2":
                                return new SequentialCommandGroup(
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("TEST blue")));

                        case "Subwoofer 5-6-7":
                                return new SequentialCommandGroup(
                                        new InstantCommand(() -> RobotContainer.getClimber().setLeftClimber(0.3)),
                                        new InstantCommand(() -> RobotContainer.getClimber().setRightClimber(0.3)),
                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(

                                                        // Shoot first piece
                                                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                        new WaitCommand(0.5),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.75)),
                                                        new WaitCommand(1.35),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                        new InstantCommand(() -> RobotContainer.getShooter().spool(0, 0)),
                                                        new InstantCommand(() -> RobotContainer.getClimber().setLeftClimber(0)),
                                                        new InstantCommand(() -> RobotContainer.getClimber().setRightClimber(0)),
                                                        new WaitCommand(0),
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                        // Intake second piece
                                                        RobotContainer.getIntakeCommand().withTimeout(6),
                                                        new DeployIntake(RobotContainer.getIntake(), false)

                                                ),
                                                new SequentialCommandGroup(

                                                        // Go to second piece
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Subwoofer-6")),
                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive()).withTimeout(0.75),

                                                        // Shoot second piece
                                                        new ParallelCommandGroup(
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("6-Stage")),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(2.75),
                                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                                new WaitCommand(0.55),
                                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                                new WaitCommand(1),
                                                                new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0, 0))
                                                        )
                                                        )
                                                )
                                        )
                                );

                        default:
                                return null;
                }
        }
}
