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
import frc.robot.commands.AlignCommands.AlignSpeaker;
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
                                                new WaitCommand(0),
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(0.6, 0.6)),
                                                new WaitCommand(0.7),
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

                        case "Subwoofer 6-7-8":
                                return new SequentialCommandGroup(
                                        new InstantCommand(() -> RobotContainer.getShooter().spool(.8, .8)),

                                        new InstantCommand(() -> RobotContainer.getClimber().setLeftClimber(0.7)),
                                        new InstantCommand(() -> RobotContainer.getClimber().setRightClimber(0.7)),
                                        //new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(35)),
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(

                                                        // Shoot first piece
                                                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                        new WaitCommand(0.85),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.75)),
                                                        new WaitCommand(0.9),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                  //      new InstantCommand(() -> RobotContainer.getShooter().spool(0, 0)),
                                                        new InstantCommand(() -> RobotContainer.getClimber().setLeftClimber(0)),
                                                        new InstantCommand(() -> RobotContainer.getClimber().setRightClimber(0)),
                                                        new WaitCommand(0),
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                        // Intake second piece
                                                        RobotContainer.getIntakeCommand().withTimeout(3.3),
                                                        new DeployIntake(RobotContainer.getIntake(), false)

                                                ),
                                                new SequentialCommandGroup(

                                                        // Go to second piece
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Subwoofer-6")),
                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(0.65),

                                                        new ParallelCommandGroup(
                                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                                new SequentialCommandGroup(
                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("6-Stage")),
                                                                        new ParallelCommandGroup(
                                                                        new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.6),
                                                                        new SequentialCommandGroup(
                                                                                new WaitCommand(0.5),
                                                                                // Shoot second peiece
                                                                                RobotContainer.getFeedCommand()
                                                                        )
                                                                        ),
                                                                        // Go to third piece
                                                                        new ParallelCommandGroup(
                                                                                new SequentialCommandGroup(
                                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Stage-7")),
                                                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(1),
                                                                                        new ParallelCommandGroup(
                                                                                                // Go to shoot third piece
                                                                                                new SequentialCommandGroup(
                                                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("7-Shoot")),
                                                                                                        new ParallelCommandGroup(
                                                                                                                new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.65),
                                                                                                                new SequentialCommandGroup(
                                                                                                                        new WaitCommand(0.5),
                                                                                                                        RobotContainer.getFeedCommand()
                                                                                                                )
                                                                                                        ),
                                                                                                        
                                                                                                        new ParallelCommandGroup(
                                                                                                                // Go to fourth piece
                                                                                                                new SequentialCommandGroup(
                                                                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Shoot-8")),
                                                                                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()),
                                                                                                                        new ParallelCommandGroup(
                                                                                                                                new SequentialCommandGroup(
                                                                                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("8-Shoot")),
                                                                                                                                        new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.75),
                                                                                                                                        RobotContainer.getFeedCommand()
                                                                                                                                ),
                                                                                                                                // Shoot fourth piece
                                                                                                                                new SequentialCommandGroup(
                                                                                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                                                                                                        new WaitCommand(0.95),
                                                                                                                                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                                                                                                        new WaitCommand(1.35),
                                                                                                                                        //new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                                                                                                        new WaitCommand(1),
                                                                                                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))
                                                                                                                              //          new InstantCommand(() -> RobotContainer.getShooter().spool(0, 0))
                                                                                                                                )
                                                                                                                        )
                                                                                                                ),
                                                                                                                new SequentialCommandGroup(
                                                                                                                        // Intake fourth piece
                                                                                                                        new WaitCommand(0.1),
                                                                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                                                                                        RobotContainer.getIntakeCommand().withTimeout(3.375),
                                                                                                                        new DeployIntake(RobotContainer.getIntake(), false)
                                                                                                                )
                                                                                                        )
                                                                                                ),
                                                                                                new SequentialCommandGroup(
                                                                                                        // Shoot third piece
                                                                                                        new WaitCommand(0),
                                                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                                                                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                                                                        new WaitCommand(1.75),
                                                                                                        //new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                                                                        new WaitCommand(1),
                                                                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0))
                                                                                            //            new InstantCommand(() -> RobotContainer.getShooter().spool(0, 0))
                                                                                                )
                                                                                        )
                                                                                ),
                                                                                new SequentialCommandGroup(
                                                                                        // Intake third piece
                                                                                        new WaitCommand(1.2),
                                                                                        RobotContainer.getIntakeCommand().withTimeout(2.875),
                                                                                        new DeployIntake(RobotContainer.getIntake(), false)
                                                                                )
                                                                        )
                                                                ),
                                                                // Shoot second piece
                                                                new SequentialCommandGroup(
                                                                        new WaitCommand(1.7),
                                                                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.85, 0.85)),
                                                                        new WaitCommand(0.9),
                                                                        //new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.4)),
                                                                        new WaitCommand(1.5)
                                                                        //new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                                        //new InstantCommand(() -> RobotContainer.getShooter().spool(0, 0))
                                                                )
                                                        )
                                                        
                                                )
                                        )
                                );

                        case "Source 4-5":
                                return new SequentialCommandGroup(
                                        // 1st shot
                                        new ParallelCommandGroup(
                                                // Forever spool
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(.8, 8)),
                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),

                                                new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.9),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(0.8),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.7))
                                                )
                                        ),
                                                        
                                        new ParallelCommandGroup(
                                                // Go to second Piece
                                                new SequentialCommandGroup(
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Source-4")),
                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(0.8),
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("4-Shoot")),
                                                        new ParallelCommandGroup(
                                                                new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                                new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(.6),
                                                                new SequentialCommandGroup(
                                                                        new WaitCommand(0.5),
                                                                        RobotContainer.getFeedCommand()
                                                                )
                                                        ),
                                                        new ParallelCommandGroup(
                                                                new SequentialCommandGroup(
                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Source-5")),
                                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(1.1),
                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("5-Shoot")),
                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                                        new ParallelCommandGroup(
                                                                                new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.9),
                                                                                new SequentialCommandGroup(
                                                                                        new WaitCommand(.7),
                                                                                        RobotContainer.getFeedCommand()
                                                                                )
                                                                        )
                                                                ),
                                                                new SequentialCommandGroup(
                                                                        // Second intake
                                                                        new WaitCommand(0.5),
                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                                        RobotContainer.getIntakeCommand().withTimeout(8)
                                                                )
                                                        )

                                                ),

                                                new SequentialCommandGroup(
                                                        new WaitCommand(2),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                        // First intake
                                                        RobotContainer.getIntakeCommand().withTimeout(7)
                                                )
                                        )
                                );
                        case "Subwoofer 1-2-3":
                                return new SequentialCommandGroup(
                                        // 1st shot
                                        new ParallelCommandGroup(
                                                // Forever spool
                                                new InstantCommand(() -> RobotContainer.getShooter().spool(.8, 8)),
                                                //new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(60)),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(0.4),
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Subwoofer-1")),
                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(0.65),
                                                        new WaitCommand(0.75),
                                                        new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(2.1),
                                                        // Shoot second shot
                                                        RobotContainer.getFeedCommand(),
                                                        new WaitCommand(0.35),
                                                                new SequentialCommandGroup(
                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                                        new ParallelCommandGroup(
                                                                                new SequentialCommandGroup(
                                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("1-2")),
                                                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(0.75)       
                                                                                ),
                                                                                RobotContainer.getIntakeCommand().withTimeout(4.75)
                                                                        ),
                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),

                                                                        new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.375),
                                                                        // Shoot third shot
                                                                        RobotContainer.getFeedCommand(),
                                                                        new WaitCommand(0.2),
                                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                                                new SequentialCommandGroup(
                                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                                                        new ParallelCommandGroup(
                                                                                                new SequentialCommandGroup(
                                                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("2-3")),
                                                                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(1)
                                                                                                ),
                                                                                                RobotContainer.getIntakeCommand().withTimeout(4)
                                                                                        ),
                                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                                                        new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.45),
                                                                                        RobotContainer.getFeedCommand()
                                                                                )
                                                        )
                                                ),
                                                new SequentialCommandGroup(
                                                        // Shoot first shot
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                        new WaitCommand(0.35),
                                                        RobotContainer.getFeedCommand(),
                                                        new WaitCommand(0.25),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                        RobotContainer.getIntakeCommand().withTimeout(3.5),
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true))
                                                )
                                        )
                                );

                        case "Subwoofer 5-6":
                                return new SequentialCommandGroup(
                                        new InstantCommand(() -> RobotContainer.getShooter().spool(.8, .8)),

                                        new InstantCommand(() -> RobotContainer.getClimber().setLeftClimber(0.7)),
                                        new InstantCommand(() -> RobotContainer.getClimber().setRightClimber(0.7)),
                                        //new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                        new InstantCommand(() -> RobotContainer.getShooter().setTargetAngle(35.5)),
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(

                                                        // Shoot first piece
                                                        new InstantCommand(() -> RobotContainer.getShooter().spool(0.8, 0.8)),
                                                        new WaitCommand(0.85),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(-0.75)),
                                                        new WaitCommand(0.8),
                                                        new InstantCommand(() -> RobotContainer.getShooter().setFeedMotor(0)),
                                                        new InstantCommand(() -> RobotContainer.getClimber().setLeftClimber(0)),
                                                        new InstantCommand(() -> RobotContainer.getClimber().setRightClimber(0)),
                                                        new WaitCommand(0),
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),

                                                        // Intake second piece
                                                        RobotContainer.getIntakeCommand().withTimeout(4),
                                                        new DeployIntake(RobotContainer.getIntake(), false)

                                                ),
                                                new SequentialCommandGroup(

                                                        // Go to second piece
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Subwoofer-5")),
                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(1.25),
                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("5-Stage")),
                                                        new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.5),
                                                        RobotContainer.getFeedCommand(),
                                                        new WaitCommand(0.35),
                                                        new ParallelCommandGroup(
                                                                new SequentialCommandGroup(
                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Stage-6")),
                                                                        new AlignNote(RobotContainer.getVision(), RobotContainer.getDrive(), RobotContainer.getIntake()).withTimeout(1),
                                                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("6-Shoot")),
                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(true)),
                                                                        // Shoot second piece
                                                                        new AlignSpeaker(RobotContainer.getDrive(), RobotContainer.getPoseEstimator()).withTimeout(0.65),
                                                                        RobotContainer.getFeedCommand()
                                                                ),
                                                                new SequentialCommandGroup(
                                                                        new InstantCommand(() -> RobotContainer.setAutoTrackSpeaker(false)),
                                                                        new WaitCommand(0.25),
                                                                        //3rd intake
                                                                        RobotContainer.getIntakeCommand().withTimeout(4.5)
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
