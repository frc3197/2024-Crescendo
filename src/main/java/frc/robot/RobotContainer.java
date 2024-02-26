package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.*;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.DetectIntake;
import frc.robot.commands.IntakeCommands.IntakeSequence;
import frc.robot.commands.IntakeCommands.SpinDirectional;
import frc.robot.commands.IntakeCommands.SpinIntake;
import frc.robot.commands.MiscCommands.AlertCommand;
import frc.robot.commands.MiscCommands.TeleopSwerve;
import frc.robot.commands.ShooterCommands.FeedToShooter;
import frc.robot.commands.ShooterCommands.ShooterElevation;
import frc.robot.commands.ShooterCommands.Spool;
import frc.robot.commands.ShooterCommands.SpoolToSpeed;
import frc.robot.subsystems.*;

public class RobotContainer {
        /* Controllers */
        private final static CommandXboxController driveController = new CommandXboxController(0);
        private final static CommandXboxController shootController = new CommandXboxController(1);

        /* Subsystems */
        private static final Drive drive = new Drive();
        private static final Intake intake = new Intake();
        private static final Vision vision = new Vision();
        private static final PoseEstimator poseEstimator = new PoseEstimator(drive);
        private static final Shooter shooter = new Shooter(poseEstimator);

        private static final Auto auto = new Auto();
        private static final Climber climber = new Climber();
        private static final AutoLookup autoLookup = new AutoLookup();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                drive.setDefaultCommand(
                                new TeleopSwerve(
                                                drive,
                                                poseEstimator,
                                                () -> driveController.getLeftY(),
                                                () -> driveController.getLeftX(),
                                                () -> -driveController.getRightX(),
                                                () -> false));

                shooter.setDefaultCommand(new ShooterElevation(shooter));

                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData(autoChooser);

                autoChooser.addOption("Middle 2p", AutoLookup.getAuto("Middle 2p"));
                autoChooser.addOption("Middle 3p-podium", AutoLookup.getAuto("Middle 3p-podium"));
                autoChooser.addOption("Stay Still", AutoLookup.getAuto("Only Shoot"));
                autoChooser.addOption("test", AutoLookup.getAuto("TEST"));

                CameraServer.startAutomaticCapture();
        }

        private void configureButtonBindings() {
                driveController.start().onTrue(new InstantCommand(() -> drive.zeroHeading()));

                shootController.rightTrigger(0.25).onTrue(new InstantCommand(() -> shooter.spool(0.85, 0.85)))
                                .onFalse(new InstantCommand(() -> shooter.spool(0.0, 0.0)));

                shootController.leftTrigger(0.25).onTrue(new InstantCommand(() -> shooter.spool(0.19, 0.11)))
                                .onFalse(new InstantCommand(() -> shooter.spool(0.0, 0.0)));

                shootController.rightBumper().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> intake.setRollers(0.5)),
                                                new InstantCommand(() -> shooter.setFeedMotor(-0.55)),
                                                new InstantCommand(() -> intake.setDirectional(-0.25))))
                                .onFalse(
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> intake.setRollers(0)),
                                                                new InstantCommand(() -> shooter.setFeedMotor(0)),
                                                                new InstantCommand(() -> intake.setDirectional(0))));

                shootController.y().onTrue(new InstantCommand(() -> shooter.setTargetAngle(55)));
                shootController.a().onTrue(new InstantCommand(() -> shooter.setTargetAngle(25)));

                shootController.povUp()
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(-0.4)),
                                                new InstantCommand(() -> climber.setRightClimber(-0.4))))
                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> climber.setLeftClimber(0)),
                                                new InstantCommand(() -> climber.setRightClimber(0))));
                shootController.povDown()
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(0.4)),
                                                new InstantCommand(() -> climber.setRightClimber(0.4))))
                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> climber.setLeftClimber(0)),
                                                new InstantCommand(() -> climber.setRightClimber(0))));

                driveController.povUp().onTrue(new DeployIntake(intake, true));
                driveController.povDown().onTrue(new DeployIntake(intake, false));

                shootController.leftBumper().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> shooter.setFeedMotor(0.4)),
                                                new InstantCommand(() -> intake.setDirectional(0.2)),
                                                new InstantCommand(() -> intake.setRollers(-0.75))))
                                .onFalse(
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> shooter.setFeedMotor(0)),
                                                                new InstantCommand(() -> intake.setDirectional(0)),
                                                                new InstantCommand(() -> intake.setRollers(0))));

                   /* 
                  driveController.a().onTrue(new InstantCommand(() ->
                  shooter.setElevationMotor(0.35))).onFalse(new InstantCommand(() ->
                  shooter.setElevationMotor(0.0)));
                  driveController.y().onTrue(new InstantCommand(() ->
                  shooter.setElevationMotor(-0.35))).onFalse(new InstantCommand(() ->
                  shooter.setElevationMotor(0.0)));
                  */
                 

                driveController.rightTrigger().onTrue(
                                getIntakeCommand());

                driveController.povRight().onTrue(new InstantCommand(() -> intake.setRollers(.5))).onFalse(new InstantCommand(() -> intake.setRollers(0)));
                
                shootController.b().onTrue(new InstantCommand(() -> shooter.setFeedMotor(0.3))).onFalse(new InstantCommand(() -> shooter.setFeedMotor(0)));

        }

        public Command getAutonomousCommand() {
                 
                return new SequentialCommandGroup(
                                new InstantCommand(
                                                () -> drive.zeroHeading()),
                                        autoChooser.getSelected()
                                                /* 
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-1")),
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-2"))*/
                                
                                );
                               // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-1"));,


        }

        public static boolean isRed() {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
        }

        public static Drive getDrive() {
                return drive;
        }

        public static Intake getIntake() {
                return intake;
        }

        public static Shooter getShooter() {
                return shooter;
        }

        public static Climber getClimber() {
                return climber;
        }

        public static Vision getVision() {
                return vision;
        }

        public static PoseEstimator getPoseEstimator() {
                return poseEstimator;
        }

        public static boolean getSpeakerAlignButton() {
                return driveController.getHID().getLeftTriggerAxis() > 0.25;
        }

        public static boolean getSpeakerElevationButton() {
                return driveController.getHID().getLeftTriggerAxis() > 0.25;
        }

        public static boolean getIntakeCancelButton() {
                return driveController.getHID().getRightBumper();
        }

        public static Command getIntakeCommand() {
                return new SequentialCommandGroup(
                                        new InstantCommand(() -> shooter.setTargetAngle(16)),
                                        new DeployIntake(intake, true),
                                        new InstantCommand(() -> intake.setRollers(0.75)),
                                                // Wait for TOF to activate
                                                new DetectIntake(intake),
                                                new ParallelCommandGroup(
                                                        new AlertCommand(driveController, true, true)
                                                                              .withTimeout(1),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(0.2),
                                                                                // Turn off rollers
                                                                                new InstantCommand(() -> intake
                                                                                                .setRollers(0)),

                                                                                new DeployIntake(intake, false),

                                                                                new InstantCommand(() -> intake
                                                                                                .setRollers(0.2)),
                                                                                new InstantCommand(() -> intake
                                                                                                .setDirectional(-0.1)),

                                                                                new FeedToShooter(shooter)
                                                                                                .withTimeout(1),

                                                                                new InstantCommand(() -> intake
                                                                                                .setRollers(0)),
                                                                                new InstantCommand(() -> intake
                                                                                                .setDirectional(0)),
                                                                                new InstantCommand(() -> shooter
                                                                                                .setFeedMotor(0)))));
        }
}
