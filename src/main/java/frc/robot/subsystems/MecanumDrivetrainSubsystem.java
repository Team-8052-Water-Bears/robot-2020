/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class MecanumDrivetrainSubsystem extends SubsystemBase {

  WPI_VictorSPX leftFront = new WPI_VictorSPX(0);
  WPI_VictorSPX leftBack = new WPI_VictorSPX(1);
  WPI_VictorSPX rightFront = new WPI_VictorSPX(2);
  WPI_VictorSPX rightBack = new WPI_VictorSPX(3);

  MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  // DIO ports, not sure which ones to use, these encoders are just for example
  Encoder lfEncoder = new Encoder(0, 1);
  Encoder rfEncoder = new Encoder(2, 3);
  Encoder lbEncoder = new Encoder(4, 5);
  Encoder rbEncoder = new Encoder(6, 7);

  // not surw what gyo we are using but here is a random one
  Gyro gyro = new ADXRS450_Gyro();

  MecanumDriveOdometry odometry;
  /**
   * Creates a new ExampleSubsystem.
   */
  public MecanumDrivetrainSubsystem() {
    super();
    odometry = new MecanumDriveOdometry(Constants.MecanumKinematics, new Rotation2d());
    //setDefaultCommand(new ManualDriveCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getHeading()), getSpeeds());
  }

  public void drive(double x, double y, double zRotation){

    Robot.x += x;
    Robot.y += y;
    Robot.zr += zRotation;
    drive.driveCartesian(x, y, zRotation);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds){
    
  }

  public MecanumDriveWheelSpeeds getSpeeds(){
    return new MecanumDriveWheelSpeeds(lfEncoder.getRate(), rfEncoder.getRate(), lbEncoder.getRate(), lbEncoder.getRate());
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.GyroReversed ? -1.0 : 1.0);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
}
