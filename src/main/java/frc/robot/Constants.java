// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ElectronicConstants.falconRPM;
import static frc.robot.Constants.ElectronicConstants.neoRPM;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
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
    public static final int[] sparkMaxIDs = new int[] {0, 0, 0, 0, 0, 0, 0};
    public static final int[] talonFXIDs = new int[] {0, 0, 0};

    // The motor controller's inversions are defined here...
    public static final boolean clockWise = false;
    public static final boolean counterClockWise = true;

    // The encoders/sensors channels are defined here...
    public static final int[] encoderChannels = new int[] {0, 0, 0};
    public static final int limitSwitchChannel = 0;
    public static final Port navxPort = SPI.Port.kMXP;
    public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;

    // The motors' RPM are defined here...
    public static final int neoRPM = 5676;
    public static final int falconRPM = 6380;
  }

  /** The chassis values are defined here... */
  public static class TankDriveConstants
  {
    // The distance per rotation is defined here...
    private static final int driveEncoderCPR = 0;
    private static final double wheelDiameterMeters = Units.inchesToMeters(0);
    public static final double chassisDistancePerRotation = 
    ((wheelDiameterMeters * Math.PI) / (double) driveEncoderCPR) * driveEncoderCPR;

    // The controllers deadbands are defined here...
    public static final double chassisDeadband = 0.052;

    // The physical maximums are defined here...
    private static final double chassisPhysicalMaxSpeed = 4;
    private static final double chassisPhysicalMaxRot = 720;

    // The drive maximums are defined here...
    public static final double chassisDriveMaxSpeed = chassisPhysicalMaxSpeed / 4;
    public static final double chassisDriveMaxRot = chassisPhysicalMaxRot / 4;
  }

  public static class IntakeConstants
  {
    // The distance of the IR detection is defined here...
    public static final double limitProximity = 0;
  }

  /** The shooter values are defined here... */
  public static class ShooterConstants
  {
    // The PID and Feedforward gains are defined here...
    public static final double shooterP = 0;
    public static final double shooterI = 0;
    public static final double shooterD = 0;
    public static final double shooterS = 0;
    public static final double shooterV = 0;

    // The shooter's setpoints are defined here...
    public static final double closeShoot = 0;
    public static final double largeShoot = 0;
  }

  /** The wrist values are defined here... */
  public static class WristConstants
  {
    // The PID and Feedforward gains are defined here...
    public static final double wristP = 0;
    public static final double wristI = 0;
    public static final double wristD = 0;
    public static final double wristS = 0;
    public static final double wristG = 0;
    public static final double wristV = 0;
    public static final double wristMaxV = 0;
    public static final double wristMaxA = 0;

    // The distance per rotation is defined here...
    public static final double wristDistancePerRotation = 2 * Math.PI;

    // The shooter's tolerance is defined here...
    public static final double wristTolerance = 0;

    // The controller's deadband is defined here...
    public static final double wristDeadband = 0;

    // The wrist's gearing is defined here...
    private static final double wristGearing = 8.75;

    // The wrist's physical maximum is defined here...
    private static final double wristPhysicalMaxRot = neoRPM / wristGearing;

    // The wrist's maximum are defined here...
    public static final double wristDriveMaxRot = wristPhysicalMaxRot / 4;

    // The wrist's setpoints are defined here...
    public static final double saveWrist = 0;
    public static final double engandeWrist = 0;
  }

  /** The climber values are defined here... */
  public static class ClimberConstants
  {
    // The PID and Feedforward gains are defined here...
    public static final double climberP = 0;
    public static final double climberI = 0;
    public static final double climberD = 0;
    public static final double climberS = 0;
    public static final double climberG = 0;
    public static final double climberV = 0;
    public static final double climberMaxV = 0;
    public static final double climberMaxA = 0;

    // The shooter's tolerance is defined here...
    public static final double climberTolerance = 0;

    // The controller's deadband is defined here...
    public static final double climberDeadband = 0;

    // The distance per pulse is defined here...
    private static final int climberEncoderCPR = 2048;
    private static final int climberGearing = 25;
    public static final double climberDistanceConversion =
    Units.feetToMeters(1/(double)climberEncoderCPR * (double)climberGearing * 1.753 * Math.PI/1 * 1/12);

    // The wrist's physical maximum is defined here...
    private static final double climberPhysicalMaxSpeed = falconRPM / climberGearing;

    // The wrist's maximum are defined here...
    public static final double climberDriveMaxSpeed = climberPhysicalMaxSpeed / 4;

    // The climber's setpoint is defined here...
    public static final double upClimber = 0;
  }
}
