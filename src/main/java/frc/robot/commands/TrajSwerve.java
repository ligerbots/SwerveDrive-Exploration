// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TrajSwerve {
	DriveTrain m_driveTrain;

	/** Creates a new TrajSwerve. */
	public TrajSwerve(DriveTrain driveTrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_driveTrain = driveTrain;
	}

	public Command getTrajCommoand (){
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
				Constants.kMaxSpeedMetersPerSecond,
				Constants.kMaxAccelerationMetersPerSecondSquared)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(m_driveTrain.getKinematics());

		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				config);

		var thetaController = new ProfiledPIDController(
				Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
				exampleTrajectory,
				m_driveTrain::getPose, // Functional interface to feed supplier
				m_driveTrain.getKinematics(),

				// Position controllers
				new PIDController(Constants.kPXController, 0, 0),
				new PIDController(Constants.kPYController, 0, 0),
				thetaController,
				m_driveTrain::setModuleStates,
				m_driveTrain);

		// Reset odometry to the starting pose of the trajectory.
		m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

		// m_driveTrain.showCurrentTrajectory(exampleTrajectory);
		return swerveControllerCommand.andThen(() -> m_driveTrain.drive(0, 0, 0, false));
	}
}
