package frc.robot.commands.MiscCommands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Drive s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private PIDController speakerAlignPID;

    private PoseEstimator poseEstimator;

    public TeleopSwerve(Drive s_Swerve, PoseEstimator poseEstimator, DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        this.poseEstimator = poseEstimator;

        this.speakerAlignPID = new PIDController(0.15, 0, 0);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // System.out.println(translationVal + "" + strafeVal);


        //TODO:: Speaker allignments
        if (RobotContainer.getSpeakerAlignButton()) {

            if (RobotContainer.isRed()) {
                rotationVal = speakerAlignPID.calculate(s_Swerve.getHeading().getDegrees()
                        - (poseEstimator.getRotationToSpeaker(RobotContainer.isRed()))) / 12;
            } else {
                rotationVal = -speakerAlignPID.calculate(s_Swerve.getHeading().getDegrees()
                        - (-poseEstimator.getRotationToSpeaker(RobotContainer.isRed()))) / 12;
            }

            if (Math.abs(rotationVal) < 0.05) {
                rotationVal = 0;
            }

            rotationVal = MathUtil.clamp(rotationVal, -0.5, 0.5);
        }

        SmartDashboard.putNumber("Rotation to Speaker", poseEstimator.getRotationToSpeaker(RobotContainer.isRed()));

        /* Drive */
        if (!DriverStation.isAutonomous()) {
            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity,
                    !robotCentricSup.getAsBoolean(),
                    true);
        }
    }
}