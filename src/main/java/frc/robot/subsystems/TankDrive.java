// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import static frc.robot.Constants.TankDriveConstants.*;
import static frc.robot.Constants.ElectronicConstants.*;

public class TankDrive extends SubsystemBase
{
  
  // The tank drive's motor controllers are defined here...
  private final CANSparkMax leaderLeft = new CANSparkMax(6, neoMotorType);
  private final CANSparkMax followLeft = new CANSparkMax(1, neoMotorType);
  private final CANSparkMax leaderRight = new CANSparkMax(3, neoMotorType);
  private final CANSparkMax followRight = new CANSparkMax(5, neoMotorType);

  // The tank drive's encoders/sensors are defined here...
  private final Encoder leftEncoder = new Encoder(2, 3);
  private final Encoder rightEncoder = new Encoder(0, 1);
  private final AHRS navx = new AHRS(navxPort);

  // The differential drive is defined here...
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leaderLeft, leaderRight);
    private final DifferentialDriveOdometry odometry;

  private final Field2d field = new Field2d();
  private final AprilTagFieldLayout aprilLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.75);
  private final Pose3d tagTestPose = aprilLayout.getTagPose(5).get();
   
  /** Creates a new Tank Drive. */
  public TankDrive()
  {

    // The motors' modes are defined here...
    leaderLeft.setIdleMode(neoBrakeMode);
    followLeft.setIdleMode(neoBrakeMode);
    leaderRight.setIdleMode(neoBrakeMode);
    followRight.setIdleMode(neoBrakeMode);

    // The motor groups are defined here...
    followLeft.follow(leaderLeft);
    followRight.follow(leaderRight);

    leaderRight.setInverted(counterClockWise);
    followRight.setInverted(counterClockWise);

    // The encoders' distance per pulse are defined here...
    leftEncoder.setDistancePerPulse(chassisDistancePerPulse);
    rightEncoder.setDistancePerPulse(chassisDistancePerPulse);

    leftEncoder.setReverseDirection(counterClockWise);
    rightEncoder.setReverseDirection(clockWise);

    // The deadband is defined here...
    differentialDrive.setDeadband(chassisDeadband);

    reset();

    odometry = new DifferentialDriveOdometry(navx.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    AutoBuilder.configureRamsete
    (this::getPose, 
    this::resetOdometry,
    this::getSpeeds,
    this::autoDrive, 
    new ReplanningConfig(),
    () ->
    {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) 
      {
        return false;
      }
      return false;
    },
    this);
    navx.resetDisplacement();
    navx.reset();
    navx.isCalibrating();
    resetOdometry(new Pose2d());


  }
  private final DifferentialDrivePoseEstimator m_poseEstimator = 
  new DifferentialDrivePoseEstimator(tankKinematics, navx.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), new Pose2d(),VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
  VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  /** Use to drive the chassis... */
  public void drive(double speedJoystick, double rotJoystick)
  {

    double speed = speedJoystick;
    double rot = rotJoystick;

    differentialDrive.arcadeDrive(speed, rot);
  }

  public void autoDrive(ChassisSpeeds tankChassisSpeeds)
  {
    DifferentialDriveWheelSpeeds tankWheelsSpeeds = tankKinematics.toWheelSpeeds(tankChassisSpeeds);
    tankWheelsSpeeds.desaturate(0.75);

    differentialDrive.tankDrive(-tankWheelsSpeeds.leftMetersPerSecond, -tankWheelsSpeeds.rightMetersPerSecond);
  }

  /** Use to get the chassis' distance... */
  public double getDistance()

  {
    // Return the process variable measurement here...
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    return (leftDistance + rightDistance) / 2;
  }

  public ChassisSpeeds getSpeeds()
  {
    return m_kinematics.toChassisSpeeds(getWheelSpeed()); // agregado por emiliano he iker 
    /* 
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());

    return tankKinematics.toChassisSpeeds(wheelSpeeds);
    */
  }

  public DifferentialDriveWheelSpeeds getWheelSpeed(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());  //lo puso emiliano he Iker :D 
  }

  /** Use to get the chassis' direction... */
  public double getDirection()
  {
    return navx.getRotation2d().getDegrees();
  }

  /** Use to reset all encoders/sensors */
  public void reset()
  {
    navx.zeroYaw();

    leftEncoder.reset();
    rightEncoder.reset();
  }

  public Pose2d getPose()
  {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose)
  {
    /* 
    reset();
    odometry.resetPosition(navx.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    */
    reset();
    odometry.resetPosition(Rotation2d.fromDegrees(navx.getYaw()),leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }
  public void megatagPosition(){
    boolean doRejectUpdate = true;
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(navx.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,1));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
  }
  public void driveToTarget(){
    double kP = .05;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= -3;
    drive(targetingForwardSpeed,0);


  }
  
  public void updateOdometry(){
        m_poseEstimator.update(navx.getRotation2d(), rightEncoder.getDistance(),leftEncoder.getDistance());
        megatagPosition();

  }
  
  
  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    updateOdometry();
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Target ID", LimelightHelpers.getFiducialID("limelight"));
    SmartDashboard.putNumber("Targed X", LimelightHelpers.getTX("limelight"));

    SmartDashboard.putNumber("Distance", getDistance());
    SmartDashboard.putNumber("Pos X", getPose().getX());
    SmartDashboard.putNumber("Pos Y", getPose().getY());
    SmartDashboard.putNumber("Direction", getDirection());
    SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
    SmartDashboard.putData("Field", field);
  }
}
