// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private final SwerveModule m_rearLeft = new SwerveModule(
      Constants.kRearLeftDriveMotorPort,
      Constants.kRearLeftTurningMotorPort,
      Constants.kRearLeftDriveEncoderPorts,
      Constants.kRearLeftTurningEncoderPorts,
      Constants.kRearLeftDriveEncoderReversed,
      Constants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.kFrontRightDriveMotorPort,
      Constants.kFrontRightTurningMotorPort,
      Constants.kFrontRightDriveEncoderPorts,
      Constants.kFrontRightTurningEncoderPorts,
      Constants.kFrontRightDriveEncoderReversed,
      Constants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      Constants.kRearRightDriveMotorPort,
      Constants.kRearRightTurningMotorPort,
      Constants.kRearRightDriveEncoderPorts,
      Constants.kRearRightTurningEncoderPorts,
      Constants.kRearRightDriveEncoderReversed,
      Constants.kRearRightTurningEncoderReversed);

  private final SwerveModule[] m_swerveModules = {
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight
  };

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(Constants.kModulePositions[0],
      Constants.kModulePositions[1], Constants.kModulePositions[2], Constants.kModulePositions[3]);

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);

  Field2d m_fieldSim = new Field2d();
  SwerveModuleState[] m_swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
  double m_yawValue = 0;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    SmartDashboard.putData("Field", m_fieldSim);
  } 

  @Override
  public void periodic() {
    m_odometry.update(new Rotation2d(getHeading()), m_swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Save rotation input for chassis rotation sim
    m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
  }

  @Override
  public void simulationPeriodic() {
    var chassisSpeed = m_kinematics.toChassisSpeeds(m_swerveModuleStates);
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

    m_yawValue += chassisRotationSpeed * 0.02;
    m_gyroSim.setAngle(-Units.radiansToDegrees(m_yawValue));

    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
}
