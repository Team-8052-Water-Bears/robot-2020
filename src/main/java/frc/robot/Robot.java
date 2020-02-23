/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.subsystems.MecanumDrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static OI oi = new OI();
  public static MecanumDrivetrainSubsystem drivetrain = new MecanumDrivetrainSubsystem();
  public static UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

  private Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("Robot.robotInit()");
    oi = new OI();
    drivetrain.setDefaultCommand(new ManualDriveCommand());
   }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    handleGameData();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    System.out.println("Robot.autonomousInit()");
    autonomousCommand = getAutonomousCommand();
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void handleGameData() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        // System.out.println("Received blue");
        break;
      case 'G':
        // Green case code
        break;
      case 'R':
        // Red case code
        break;
      case 'Y':
        // Yellow case code
        break;
      default:
        // This is corrupt data
        break;
      }
      SmartDashboard.putString("Game Data", gameData);
    } else {
      // Code for no data received yet
    }
  }

  // @Incomplete: Tune constants
  public Command getAutonomousCommand(){

    TrapezoidProfile.Constraints constraints =
    new TrapezoidProfile.Constraints(1, 1);

    TrajectoryConfig config =
     new TrajectoryConfig(AutoConstants.MaxVelocity, AutoConstants.MaxAcceleration)
     .setKinematics(Constants.MecanumKinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d()), 
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),
      new Pose2d(3, 0, new Rotation2d()),
      config);

      MecanumControllerCommand command = new MecanumControllerCommand
      (trajectory, 
      Robot.drivetrain::getPose, 
      Constants.MecanumKinematics,
      new PIDController(0, 0, 0),
      new PIDController(0, 0, 0),
      new ProfiledPIDController(0, 0, 0, constraints), 
      AutoConstants.MaxWheelVelocity,
      Robot.drivetrain::setSpeeds,
      Robot.drivetrain);
    return command.andThen(() -> System.out.println("Finished"));
  }
}
