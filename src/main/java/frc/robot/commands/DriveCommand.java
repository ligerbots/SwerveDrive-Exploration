// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {

	DriveTrain m_driveTrain;
	XboxController m_driverController;
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

	boolean m_fieldRelative;
	/** Creates a new DriveCommand. */
	public DriveCommand(DriveTrain driveTrain, XboxController driveController, boolean fieldRelative) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_driveTrain = driveTrain;
		m_driverController = driveController;
		m_fieldRelative = fieldRelative;
		addRequirements(m_driveTrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.02))
				* Constants.kMaxSpeedMetersPerSecond;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.02))
				* Constants.kMaxSpeedMetersPerSecond;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX(), 0.02))
				* Constants.kMaxChassisAngularSpeedRadiansPerSecond;

		m_driveTrain.drive(xSpeed, ySpeed, rot, m_fieldRelative);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
