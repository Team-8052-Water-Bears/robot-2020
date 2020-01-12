/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  WPI_VictorSPX leftMaster = new WPI_VictorSPX(0);
  WPI_VictorSPX leftSlave = new WPI_VictorSPX(1);
  WPI_VictorSPX rightMaster = new WPI_VictorSPX(2);
  WPI_VictorSPX rightSlave = new WPI_VictorSPX(3);



  DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  /**
   * Creates a new ExampleSubsystem.
   */
  public DrivetrainSubsystem() {
      leftSlave.follow(leftMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double move, double turn){
    drive.arcadeDrive(move, turn);
  }
}
