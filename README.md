# Bibble✨
FIRST Robotics Competition code for the 2024 season CRESCENDO. 🎼

The code in this project was created for Bibble Pelón, robot of Team 4403 ROULT of PrepaTec Campus Laguna. Bibble Pelón was designed to participate in the FIRST Robotics Competition (FRC) 2024 season, CRESCENDO ©️.

This project contains the code used to control different parts of the robot. It contains subsystems which organize functions of the robot into classes that contain all of the neccesary commands to drive the robot.

Auto 🏆

Bibble Pelón uses odometry obtained by a navX-MXP and ThroughBore Encoders to plan autonomous sequences using PathPlanner.

Subsystems 📋

Bibble Pelón has 6 different subsystems that control different mechanisms on the robot. These subsystems are:
| Subsystem | Function |
|--------|--------|
|[Drivetrain](https://github.com/ROULT4403/FRC_2024/blob/breakdowm/src/main/java/frc/robot/subsystems/TankDrive.java)| 	Controls Bibble Pelón movement. Contains sensors used in autonomous, as well as the AutoBuilder that is used by PathPlanner to build the autonomous routines.|
|[Intake](https://github.com/ROULT4403/FRC_2024/blob/breakdowm/src/main/java/frc/robot/subsystems/Intake.java)| 	Used to collect Notes as well as feeding them to the shooter during a match , using a Falcon 500.|
|[Wrist](https://github.com/ROULT4403/FRC_2024/blob/breakdowm/src/main/java/frc/robot/subsystems/Wrist.java)| 	Used to move the Intake joint using a Spark Max + NEO Brushless Motor|
|[Shooter](https://github.com/ROULT4403/FRC_2024/blob/breakdowm/src/main/java/frc/robot/subsystems/shooter.java)| 	Used to shoot notes out at a fixed angle.|
|[Climber](https://github.com/ROULT4403/FRC_2024/blob/breakdowm/src/main/java/frc/robot/subsystems/Climber.java)| 	Used during the endagme period of a match, it hangs Bibble Pelón using 2 Falcon 500. To aid the drivers, the command is stopped via RaceCommandGroup, checking if the physical limit switch is pressed or the encoder position is greater than a certain point.|
|[AutoConfig](https://github.com/ROULT4403/FRC_2024/blob/breakdowm/src/main/java/frc/robot/subsystems/TankDrive.java)| 	Simply used to improve the readability of the code. This subsystem declares the NamedCommands used with PathPlanner.|


Convenience features during the match📷

Bibble Pelón is equipped with 2 cameras connected to the roboRIO, providing a better view of the Intake and the whole field.

Haptic indicators are used to aid drivers, as the controller will rumble when the shooter gets to its target velocity.
