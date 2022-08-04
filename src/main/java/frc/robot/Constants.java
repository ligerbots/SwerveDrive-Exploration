// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5;
    // Distance between front and back wheels on robot

    public static final Translation2d kFrontLeftModulePosition = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d kFrontRightModulePosition = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d kBackLeftModulePosition = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d kBackRightModulePosition = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    public static final Translation2d[] kModulePositions = {
            kFrontLeftModulePosition,
            kFrontRightModulePosition,
            kBackLeftModulePosition,
            kBackRightModulePosition
    };


    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kRearLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightTurningEncoderPorts = new int[] {6, 7};

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
    public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
    public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
    public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final int kEncoderCPR = 2048;
    public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / kEncoderCPR;

    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;


    public static final double kMaxSpeedMetersPerSecond = 8;

    // public static final double kMaxChassisAngularSpeedRadiansPerSecond = Math.PI / 8.0;
    public static final double kMaxChassisAngularSpeedRadiansPerSecond = 2*Math.PI;
}
