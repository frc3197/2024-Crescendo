package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;

public class Auto extends Command {

    private Drive drive;
    private PoseEstimator poseEstimator;

    public Auto() {
        drive = RobotContainer.getDrive();        
        poseEstimator = RobotContainer.getPoseEstimator();        

        AutoBuilder.configureHolonomic(
                drive::getPose,
                drive::setPose,
                drive::getSpeeds,
                drive::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(
                                Constants.auto.pathplannerDriveKP, // Translation PID
                                Constants.auto.pathplannerDriveKI,
                                Constants.auto.pathplannerDriveKD),
                        new PIDConstants(
                                Constants.auto.pathplannerRotKP, // Rotation PID
                                Constants.auto.pathplannerRotKI,
                                Constants.auto.pathplannerRotKD),
                        Constants.Swerve.maxSpeed, // MAX SPEED
                        0.5, // ROBOT RADIUS
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        //return true;
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive);

        addRequirements(drive);
    }
}
