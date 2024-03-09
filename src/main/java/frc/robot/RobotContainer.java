package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.*;
import frc.robot.commands.AlignCommands.AlignNote;
import frc.robot.commands.IntakeCommands.DeployIntake;
import frc.robot.commands.IntakeCommands.DetectIntake;
import frc.robot.commands.MiscCommands.AlertCommand;
import frc.robot.commands.MiscCommands.TeleopSwerve;
import frc.robot.commands.ShooterCommands.FeedToShooter;
import frc.robot.commands.ShooterCommands.ShooterElevation;
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
        private static final Shooter shooter = new Shooter(poseEstimator, drive);

        private static final Auto auto = new Auto();
        private static final Climber climber = new Climber();
        private static final AutoLookup autoLookup = new AutoLookup();

        private final SendableChooser<Command> autoChooser;
        private static SendableChooser<Pose2d> poseChooser;

        private static boolean autoTrackSpeaker = false;

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
                poseChooser = new SendableChooser<Pose2d>();

                SmartDashboard.putData(autoChooser);
                SmartDashboard.putData(poseChooser);

                autoChooser.addOption("Middle 2p", AutoLookup.getAuto("Middle 2p"));

                autoChooser.addOption("Middle 3p-podium", AutoLookup.getAuto("Middle 3p-podium"));
                autoChooser.addOption("Middle 3p-amp", AutoLookup.getAuto("Middle 3p-amp"));
                autoChooser.addOption("Middle 4p", AutoLookup.getAuto("Middle 4p"));
                autoChooser.addOption("Amp - 2p", AutoLookup.getAuto("2p amp"));

                autoChooser.addOption("Stay Still", AutoLookup.getAuto("Only Shoot"));
                autoChooser.addOption("test", AutoLookup.getAuto("TEST"));
                autoChooser.addOption("test blue", AutoLookup.getAuto("test2"));
                
                autoChooser.addOption("Subwoofer 5-6-7", AutoLookup.getAuto("Subwoofer 5-6-7"));

                CameraServer.startAutomaticCapture();

                poseChooser.addOption("REDMID", new Pose2d(new Translation2d(15.11, 5.53), new Rotation2d(180)));
                poseChooser.addOption("REDAMP", new Pose2d(new Translation2d(15.86, 6.68), new Rotation2d(180)));
                poseChooser.addOption("REDSOURCE", new Pose2d(new Translation2d(15.82, 4.43), new Rotation2d(180)));
  
                poseChooser.addOption("BLUEMID", new Pose2d(new Translation2d(1.34, 5.56), new Rotation2d(180)));

                poseChooser.addOption("0, 0", new Pose2d(new Translation2d(0, 0), new Rotation2d(180)));
                poseChooser.addOption("16.5, 0", new Pose2d(new Translation2d(16.5, 0), new Rotation2d(180)));

                

        }

        private void configureButtonBindings() {
                driveController.start().onTrue(new InstantCommand(() -> drive.zeroHeading()));

                shootController.rightTrigger(0.25).onTrue(new InstantCommand(() -> shooter.spool(0.8, 0.8)))
                                .onFalse(new InstantCommand(() -> shooter.spool(0.0, 0.0)));

                shootController.leftTrigger(0.25).onTrue(new InstantCommand(() -> shooter.spool(0.24, 0.16)))
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





                // TODO TESTING ONLY FOR SHOOT ON ONE CONTROLLER
                driveController.x().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> intake.setRollers(0.5)),
                                                new InstantCommand(() -> shooter.setFeedMotor(-0.55)),
                                                new InstantCommand(() -> intake.setDirectional(-0.25))))
                                .onFalse(
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> intake.setRollers(0)),
                                                                new InstantCommand(() -> shooter.setFeedMotor(0)),
                                                                new InstantCommand(() -> intake.setDirectional(0))));
                driveController.a().onTrue(new InstantCommand(() -> shooter.spool(0.8, 0.8)))
                                .onFalse(new InstantCommand(() -> shooter.spool(0.0, 0.0)));









                shootController.y().onTrue(new InstantCommand(() -> shooter.setTargetAngle(52)));
                shootController.a().onTrue(new InstantCommand(() -> shooter.setTargetAngle(25)));

                shootController.povUp()
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(-0.7)),
                                                new InstantCommand(() -> climber.setRightClimber(-0.7))))
                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> climber.setLeftClimber(0)),
                                                new InstantCommand(() -> climber.setRightClimber(0))));
                shootController.povUpLeft()
                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(-0.7)),
                                                new InstantCommand(() -> climber.setRightClimber(-0.7))))
                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> climber.setLeftClimber(0)),
                                                new InstantCommand(() -> climber.setRightClimber(0))));

                shootController.povUpRight()
                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(-0.7)),
                                                new InstantCommand(() -> climber.setRightClimber(-0.7))))
                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> climber.setLeftClimber(0)),
                                                new InstantCommand(() -> climber.setRightClimber(0))));

                shootController.povDownLeft()
                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(1)),
                                                new InstantCommand(() -> climber.setRightClimber(1))))
                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> climber.setLeftClimber(0)),
                                                new InstantCommand(() -> climber.setRightClimber(0))));

                shootController.povDownRight()
                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(1)),
                                                new InstantCommand(() -> climber.setRightClimber(1))))
                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> climber.setLeftClimber(0)),
                                                new InstantCommand(() -> climber.setRightClimber(0))));

                shootController.povDown()
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> climber.setLeftClimber(1)),
                                                new InstantCommand(() -> climber.setRightClimber(1))))
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

                shootController.x().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> shooter.setFeedMotor(0.4)),
                                                new InstantCommand(() -> intake.setDirectional(0.2)),
                                                new InstantCommand(() -> intake.setRollers(-0.75)),
                                                new WaitCommand(0.05),
                                                new InstantCommand(() -> shooter.setFeedMotor(-0.4)),
                                                new InstantCommand(() -> intake.setDirectional(-0.2)),
                                                new InstantCommand(() -> intake.setRollers(0.75)),
                                                new WaitCommand(0.12),
                                                new InstantCommand(() -> shooter.setFeedMotor(0)),
                                                new InstantCommand(() -> intake.setDirectional(0)),
                                                new InstantCommand(() -> intake.setRollers(0)))
                                                );

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

                driveController.povRight().onTrue(new InstantCommand(() -> intake.setRollers(.5)))
                                .onFalse(new InstantCommand(() -> intake.setRollers(0)));

                shootController.b().onTrue(new InstantCommand(() -> shooter.setFeedMotor(0.3)))
                                .onFalse(new InstantCommand(() -> shooter.setFeedMotor(0)));

                driveController.b().whileTrue(new AlignNote(vision, drive));

                // TODO remove these temporary deflector commands

                shootController.povLeft().onTrue(new InstantCommand(() -> shooter.setDeflectorMotor(0.2))).onFalse(new InstantCommand(() -> shooter.setDeflectorMotor(0)));
                shootController.povRight().onTrue(new InstantCommand(() -> shooter.setDeflectorMotor(-0.2))).onFalse(new InstantCommand(() -> shooter.setDeflectorMotor(0)));
        }

        public Command getAutonomousCommand() {
                drive.setOdometry(getInitialPose());
                poseEstimator.setPose(getInitialPose());

                System.out.println(getInitialPose().getX() + ", " + getInitialPose().getY() + ", " + getInitialPose().getRotation());


                

                return new SequentialCommandGroup(
                                new InstantCommand(
                                                () -> drive.zeroHeading()),
                                autoChooser.getSelected()
                /*
                 * AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-1")),
                 * AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center 3p-2"))
                 */

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
                return driveController.getHID().getLeftTriggerAxis() > 0.25 || autoTrackSpeaker;
        }

        public static boolean getIntakeCancelButton() {
                return driveController.getHID().getRightBumper();
        }

        public static Pose2d getInitialPose() {
                return poseChooser.getSelected();
        }

        public static void setAutoTrackSpeaker(boolean value) {
                autoTrackSpeaker = value;
        }

        public static Command getIntakeCommand() {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> shooter.setTargetAngle(18.5)),
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

                                                                new FeedToShooter(shooter),
                                                                
                                                                //new InstantCommand(() -> shooter.setFeedMotor(0.25)),
                                                                //new WaitCommand(0.175),

                                                                new InstantCommand(() -> intake
                                                                                .setRollers(0)),
                                                                new InstantCommand(() -> intake
                                                                                .setDirectional(0)),
                                                                new InstantCommand(() -> shooter
                                                                                .setFeedMotor(0)))));
        }
}
