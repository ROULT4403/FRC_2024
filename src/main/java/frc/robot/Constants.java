// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  /** The IDs of the driver's controllers are defined here... */
  public static class OperatorConstants
  {
    // The controllers port are defined here...
    public static final int[] controllersPort = new int[] {0, 1};
  }

  /** The electronic values are defined here... */
  public static class ElectronicConstants
  {
    // The motor's type are defined here...
    public static final MotorType neoMotorType = MotorType.kBrushless;

    // The motor's mode are defined here...
    public static final NeutralModeValue falconBrakeMode = NeutralModeValue.Brake;
    public static final IdleMode neoBrakeMode = IdleMode.kBrake;
    public static final IdleMode neoCoastMode = IdleMode.kCoast;

    // The motor controller's IDs are defined here...
    public static final int[] sparkMaxIDs = new int[] {2, 1, 4, 7, 5, 6, 3};
    public static final int[] talonFXIDs = new int[] {8, 9, 10};

    // The motor controller's inversions are defined here...
    public static final boolean clockWise = false;
    public static final boolean counterClockWise = true;

    // The encoders/sensors channels are defined here...
    public static final int[] encoderChannels = new int[] {7, 8, 5, 6, 4};
    public static final int[] limitSwitchChannel = new int[] {0, 1};
    public static final Port navxPort = SPI.Port.kMXP;

    // The motors' RPM are defined here...
    public static final int neoRPM = 5676;
    public static final int falconRPM = 6380;
  }

  /** The chassis values are defined here... */
  public static class TankDriveConstants
  {
    // The distance per pulse is defined here...
    private static final double encoderCPR = 2048;
    private static final double wheelDiameterMeters = Units.inchesToMeters(6);
    public static final double chassisDistancePerPulse = (wheelDiameterMeters * Math.PI) / encoderCPR;

    public static final DifferentialDriveKinematics tankKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(29));

    // The controllers deadbands are defined here...
    public static final double chassisDeadband = 0.1;
  }

  /** The wrist values are defined here... */
  public static class WristConstants
  {
    // The distance per rotation is defined here...
    public static final double wristDistancePerRotation = 360;
  }

  /** The climber values are defined here... */
  public static class ClimberConstants
  {
    // The distance per pulse is defined here...
    private static final int climberEncoderCPR = 2048;
    private static final int climberGearing = 50;
    public static final double climberPulse = climberEncoderCPR * climberGearing;
    public static final double climberDistanceConversion = Math.PI * 1.753;
  }
}
