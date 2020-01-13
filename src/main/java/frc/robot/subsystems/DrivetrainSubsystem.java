/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrivetrainSubsystem extends SubsystemBase {

  WPI_VictorSPX leftFront = new WPI_VictorSPX(0);
  WPI_VictorSPX leftBack = new WPI_VictorSPX(1);
  WPI_VictorSPX rightFront = new WPI_VictorSPX(2);
  WPI_VictorSPX rightBack = new WPI_VictorSPX(3);

  MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  /**
   * Creates a new ExampleSubsystem.
   */
  public DrivetrainSubsystem() {
    super();
    //setDefaultCommand(new ManualDriveCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double x, double y, double zRotation){

    Robot.x += x;
    Robot.y += y;
    Robot.zr += zRotation;
    drive.driveCartesian(x, y, zRotation);
  }
}
