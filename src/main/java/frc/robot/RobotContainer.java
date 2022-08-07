// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.trajFollowing;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.PathPlanner;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// A split-stick arcade command, with forward/backward controlled by the left
		// hand, and turning controlled by the right.

		// Note: X, Y, and rot for Throttle/Strafe/Rotation is inverted to match WPILib
		// coordinate system
		var driveCommand = new RunCommand(
				() -> m_robotDrive.drive(
						m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond,
						-m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond,
						-m_driverController.getRightX() * DriveConstants.kMaxChassisAngularSpeedRadiansPerSecond,
						true));

		driveCommand.addRequirements(m_robotDrive);

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		m_robotDrive.setDefaultCommand(driveCommand);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		PIDController xController = new PIDController(2, 0, 0);
		PIDController yController = new PIDController(2, 0, 0);
		ProfiledPIDController thetaController = new ProfiledPIDController(50, 0, 0,
				new TrapezoidProfile.Constraints(4*Math.PI, 4*Math.PI));

		var traj = PathPlanner.loadPath("Test", 15.0, 8.0);
		m_robotDrive.resetOdometry(traj.getInitialPose());

		var autonomousCommand = new trajFollowing(m_robotDrive,
				traj, () -> m_robotDrive.getPose(), Constants.DriveConstants.kDriveKinematics, xController, yController,
				thetaController, (states) -> {
					m_robotDrive.setModuleStates(states);
				}, m_robotDrive).andThen(() -> m_robotDrive.stop());
		return autonomousCommand;
	}

	public DriveSubsystem getDrive(){
		return m_robotDrive;
	}
}
