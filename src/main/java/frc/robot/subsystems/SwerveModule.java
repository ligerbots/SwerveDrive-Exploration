// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {

    private MotorController m_driveMotor;
    private MotorController m_turningMotor;

    private Encoder m_driveEncoder;
    private Encoder m_turningEncoder;

    private EncoderSim m_driveEncoderSim;
    private EncoderSim m_turningEncoderSim;

    private PIDController m_drivePIDController = new PIDController(1, 0, 0);

    private ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    Constants.kModuleMaxAngularVelocity, Constants.kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    // Using FlywheelSim as a stand-in for a simple motor
    private final FlywheelSim m_turnMotorSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(Constants.kvVoltSecondsPerRadian, Constants.kaVoltSecondsSquaredPerRadian),
        Constants.kTurnMotorGearbox,
        Constants.kTurnGearRatio
    );

    private final FlywheelSim m_driveMotorSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveMotorGearbox,
        Constants.kDriveGearRatio
    );

    private double m_driveOutput;
    private double m_turnOutput;

    private double m_simTurnEncoderDistance;
    public SwerveModule(
        int driveMotorChannel,
        int turningMotorChannel,
        int[] driveEncoderPorts,
        int[] turningEncoderPorts,
        boolean driveEncoderReversed,
        boolean turningEncoderReversed) {
  
      m_driveMotor = new Spark(driveMotorChannel);
      m_turningMotor = new Spark(turningMotorChannel);
  
      this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);
  
      this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);
  
      this.m_driveEncoderSim = new EncoderSim(m_driveEncoder);
  
      this.m_turningEncoderSim = new EncoderSim(m_turningEncoder);
  
      // Set the distance per pulse for the drive encoder. We can simply use the
      // distance traveled for one rotation of the wheel divided by the encoder
      // resolution.
      m_driveEncoder.setDistancePerPulse(Constants.kDriveEncoderDistancePerPulse);
  
      // Set whether drive encoder should be reversed or not
      m_driveEncoder.setReverseDirection(driveEncoderReversed);
  
      // Set the distance (in this case, angle) per pulse for the turning encoder.
      // This is the the angle through an entire rotation (2 * wpi::math::pi)
      // divided by the encoder resolution.
      m_turningEncoder.setDistancePerPulse(Constants.kTurningEncoderDistancePerPulse);
  
      // Set whether turning encoder should be reversed or not
      m_turningEncoder.setReverseDirection(turningEncoderReversed);
  
      // Limit the PID Controller's input range between -pi and pi and set the input
      // to be continuous.
      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));
    
        // Calculate the drive output from the drive PID controller.
        m_driveOutput =
            m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
    
        // Calculate the turning motor output from the turning PID controller.
        m_turnOutput =
            m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());
    
        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(m_driveOutput);
        m_turningMotor.set(m_turnOutput);
    }

    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {
        m_driveEncoder.reset();
        m_turningEncoder.reset();
    }

    /** Simulate the SwerveModule */
    public void simulationPeriodic(double dt) {
        m_turnMotorSim.setInputVoltage(m_turnOutput / Constants.kMaxModuleAngularSpeedRadiansPerSecond * RobotController.getInputVoltage());
        m_driveMotorSim.setInputVoltage(m_driveOutput / Constants.kMaxSpeedMetersPerSecond * RobotController.getInputVoltage());

        m_turnMotorSim.update(dt);
        m_driveMotorSim.update(dt);

        // Calculate distance traveled using RPM * dt
        m_simTurnEncoderDistance += m_turnMotorSim.getAngularVelocityRadPerSec() * dt;
        m_turningEncoderSim.setDistance(m_simTurnEncoderDistance);
        m_turningEncoderSim.setRate(m_turnMotorSim.getAngularVelocityRadPerSec());

        m_driveEncoderSim.setRate(m_driveMotorSim.getAngularVelocityRadPerSec());
    }
}
