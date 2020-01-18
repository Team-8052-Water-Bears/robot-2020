/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

// Use data from vision coprocessor to look at the nearest powercell.
// This is all untested, and I'm sure that it doesn't work.
public class LookAtBallCommand extends CommandBase {

  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;
  private final double tolerance = 5;
  private PIDController controller;
  private NetworkTable visionTable;
  private NetworkTableEntry targetAngleEntry;

  /**
   * Creates a new LookAtBallCommand.
   */
  public LookAtBallCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
    controller = new PIDController(kP, kI, kD);
    controller.setTolerance(tolerance);
    visionTable = NetworkTableInstance.getDefault().getTable(Constants.VisonTableKey);
    targetAngleEntry = visionTable.getEntry("target_angle");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroAngle = Robot.drivetrain.getHeading();
    double setPoint = gyroAngle + targetAngleEntry.getDouble(0.0);
    controller.setSetpoint(setPoint);
    //controller.enableContinuousInput(0, 360);
    double calculated = controller.calculate(gyroAngle);
    // Since only heading is affected, maybe the driver should still
    // have control of the position of the robot?
    Robot.drivetrain.drive(0, 0, calculated);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
