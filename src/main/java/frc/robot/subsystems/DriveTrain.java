// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

	private final SwerveModule m_frontLeft = new SwerveModule(
			Constants.kFrontLeftDriveMotorPort,
			Constants.kFrontLeftTurningMotorPort,
			Constants.kFrontLeftDriveEncoderPorts,
			Constants.kFrontLeftTurningEncoderPorts,
			Constants.kFrontLeftDriveEncoderReversed,
			Constants.kFrontLeftTurningEncoderReversed);

	private final SwerveModule m_backLeft = new SwerveModule(
			Constants.kBackLeftDriveMotorPort,
			Constants.kBackLeftTurningMotorPort,
			Constants.kBackLeftDriveEncoderPorts,
			Constants.kBackLeftTurningEncoderPorts,
			Constants.kBackLeftDriveEncoderReversed,
			Constants.kBackLeftTurningEncoderReversed);

	private final SwerveModule m_frontRight = new SwerveModule(
			Constants.kFrontRightDriveMotorPort,
			Constants.kFrontRightTurningMotorPort,
			Constants.kFrontRightDriveEncoderPorts,
			Constants.kFrontRightTurningEncoderPorts,
			Constants.kFrontRightDriveEncoderReversed,
			Constants.kFrontRightTurningEncoderReversed);

	private final SwerveModule m_backRight = new SwerveModule(
			Constants.kBackRightDriveMotorPort,
			Constants.kBackRightTurningMotorPort,
			Constants.kBackRightDriveEncoderPorts,
			Constants.kBackRightTurningEncoderPorts,
			Constants.kBackRightDriveEncoderReversed,
			Constants.kBackRightTurningEncoderReversed);

	private final SwerveModule[] m_swerveModules = {
			m_frontLeft,
			m_frontRight,
			m_backLeft,
			m_backRight
	};

	// The gyro sensor
	private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
	private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);

	public final SwerveDriveKinematics m_kDriveKinematics = new SwerveDriveKinematics(
			Constants.kFrontLeftModulePosition,
			Constants.kFrontRightModulePosition,
			Constants.kBackLeftModulePosition,
			Constants.kBackRightModulePosition);

	// Odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kDriveKinematics, m_gyro.getRotation2d());

	Pose2d[] m_modulePose = {
			new Pose2d(),
			new Pose2d(),
			new Pose2d(),
			new Pose2d()
	};

	Field2d m_fieldSim = new Field2d();

	// Simulation variables
	private double m_invertRotationInput = -1;
	private double m_lastRotationSign;
	private double m_yawValue;

	/** Creates a new DriveTrain. */
	public DriveTrain() {
		SmartDashboard.putData("Field", m_fieldSim);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// Update the odometry in the periodic block
		m_odometry.update(
				new Rotation2d(getHeading()),
				m_frontLeft.getState(),
				m_frontRight.getState(),
				m_backLeft.getState(),
				m_backRight.getState());

		// Update the poses for the swerveModules. Note that the order of rotating the
		// position and then
		// adding the translation matters
		for (int i = 0; i < m_swerveModules.length; i++) {
			var modulePositionFromChassis = Constants.kModulePositions[i]
					.rotateBy(new Rotation2d(getHeading()))
					.plus(getPose().getTranslation());

			// Module's heading is it's angle relative to the chassis heading
			m_modulePose[i] = new Pose2d(modulePositionFromChassis,
					m_swerveModules[i].getState().angle.plus(getPose().getRotation()));
		}

		m_fieldSim.setRobotPose(getPose());
		// m_fieldSim.getObject("Swerve Modules").setPoses(m_modulePose);
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(pose, m_gyro.getRotation2d());
		resetEncoders();
	}

	public SwerveDriveKinematics getKinematics(){
		return m_kDriveKinematics;
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		// Save rotation input for chassis rotation sim

		var swerveModuleStates = m_kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, Constants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_backLeft.setDesiredState(desiredStates[2]);
		m_backRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_backLeft.resetEncoders();
		m_backRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return m_gyro.getRotation2d().getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
	}

	/**
	 * Plot the current trajectory using Field2d
	 */
	public void showCurrentTrajectory(Trajectory trajectory) {
		var trajectoryStates = new ArrayList<Pose2d>();

		trajectoryStates.addAll(trajectory.getStates().stream()
				.map(state -> state.poseMeters)
				.collect(Collectors.toList()));

		m_fieldSim.getObject("Trajectory").setPoses(trajectoryStates);
	}

	@Override
	public void simulationPeriodic() {
		m_frontLeft.simulationPeriodic(0.02);
		m_frontRight.simulationPeriodic(0.02);
		m_backLeft.simulationPeriodic(0.02);
		m_backRight.simulationPeriodic(0.02);

		SwerveModuleState[] moduleStates = {
				m_frontLeft.getState(),
				m_frontRight.getState(),
				m_backLeft.getState(),
				m_backRight.getState()
		};

		var chassisSpeed = m_kDriveKinematics.toChassisSpeeds(moduleStates);
		double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

		m_yawValue += chassisRotationSpeed * 0.02;
		m_gyroSim.setAngle(-Units.radiansToDegrees(m_yawValue));
	}
}