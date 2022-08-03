// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class SwerveModule {

    Spark m_driveMotor, m_turnMotor;

    Encoder m_driveMotorEncoder, m_turnMotorEncoder;

    EncoderSim m_driveMotorEncoderSim, m_turnMotorEncoderSim;

    PIDController m_drivePIDController = new PIDController(1, 0, 0);

    PIDController m_turnPIDController = new PIDController(1, 0, 0);

    FlywheelSim m_driveMotorSim = new FlywheelSim(DCMotor.getNEO(1), 6.12,
    (1.0 / 12.0) * (Units.lbsToKilograms(120.0) / 4) * Units.inchesToMeters(2 * 2));


    public SwerveModule(){
        
    }

}
