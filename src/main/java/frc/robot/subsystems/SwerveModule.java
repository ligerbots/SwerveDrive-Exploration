// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule extends SubsystemBase {

    Spark m_driveMotor, m_turnMotor;

    Encoder m_driveEncoder, m_turnEncoder;

    EncoderSim m_driveEncoderSim, m_turnEncoderSim;

    PIDController m_drivePIDController = new PIDController(12, 0, 0);

    PIDController m_turnPIDController = new PIDController(0.08, 0, 0);

    FlywheelSim m_driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.12,
            (1.0 / 12.0) * (Units.lbsToKilograms(120.0) / 4) * Units.inchesToMeters(2 * 2));

    FlywheelSim m_turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0,
            (1.0 / 12.0) * (Units.lbsToKilograms(120.0) / 4) * Units.inchesToMeters(1.5 * 1.5));

    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int[] driveEncoderPorts,
            int[] turningEncoderPorts,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed) {

        m_driveMotor = new Spark(driveMotorChannel);
        m_turnMotor = new Spark(turningMotorChannel);

        m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);

        m_turnEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);

        m_driveEncoderSim = new EncoderSim(m_driveEncoder);

        m_turnEncoderSim = new EncoderSim(m_turnEncoder);

        m_driveEncoder.setDistancePerPulse(Constants.kDriveEncoderDistancePerPulse);

        m_driveEncoder.setReverseDirection(driveEncoderReversed);

        m_turnEncoder.setDistancePerPulse(Constants.kTurningEncoderDistancePerPulse);

        // Set whether turning encoder should be reversed or not
        m_turnEncoder.setReverseDirection(turningEncoderReversed);

        m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        

    }


    public void resetEncoders() {
        m_driveEncoder.reset();
        m_turnEncoder.reset();
    }
}
