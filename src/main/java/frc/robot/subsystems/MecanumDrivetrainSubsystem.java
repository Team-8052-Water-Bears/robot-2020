/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.analog.adis16448.frc.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class MecanumDrivetrainSubsystem extends SubsystemBase {

  PWMVictorSPX frontLeftController = new PWMVictorSPX(0);
  PWMVictorSPX rearRightController = new PWMVictorSPX(1);
  PWMVictorSPX rearLeftController = new PWMVictorSPX(2);
  PWMVictorSPX frontRightController = new PWMVictorSPX(3);

  PIDController rearLeftPID = new PIDController(0, 0, 0);
  PIDController rearRightPID = new PIDController(0, 0, 0);
  PIDController frontLeftPID = new PIDController(0, 0, 0);
  PIDController frontRightPID = new PIDController(0, 0, 0);

  MecaDrive drive = new MecaDrive(frontLeftController, rearLeftController, frontRightController, rearRightController);

  ADIS16448_IMU imu = new ADIS16448_IMU(IMUAxis.kZ, SPI.Port.kMXP, 5);

  MecanumDriveOdometry odometry;

  /**
   * Creates a new ExampleSubsystem.
   */
  public MecanumDrivetrainSubsystem() {
    super();
    odometry = new MecanumDriveOdometry(Constants.MecanumKinematics, new Rotation2d());
    // setDefaultCommand(new ManualDriveCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getHeading()), getSpeeds());
  }

  // Called periodically when ManualDriveCommand is running
  public void drive(double x, double y, double zRotation) {
    drive.driveCartesian(x, y, zRotation, 0.0);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

  }

  public MecanumDriveWheelSpeeds getSpeeds() {
    return new MecanumDriveWheelSpeeds(getEncoder(MotorType.kFrontLeft).getRate(), getEncoder(MotorType.kFrontRight).getRate(),
    getEncoder(MotorType.kRearLeft).getRate(), getEncoder(MotorType.kRearRight).getRate());
  }

  public Encoder getEncoder(MotorType mt){
    return drive.encoders[mt.value];
  }

  public double getHeading() {
    return Math.IEEEremainder(imu.getAngle(), 360) * (Constants.GyroReversed ? -1.0 : 1.0);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
}

class MecaDrive extends MecanumDrive {

  public Encoder[] encoders = new Encoder[] { new Encoder(0, 1), new Encoder(2, 3), new Encoder(4, 5), new Encoder(6, 7), };
  double Kp = 0.3, Ki = 0, Kd = 0;
  SpeedController[] speedControllers;
  PIDController[] controllers = new PIDController[] { new PIDController(Kp, Ki, Kd), new PIDController(Kp, Ki, Kd),
      new PIDController(Kp, Ki, Kd), new PIDController(Kp, Ki, Kd), };


  double[] inputSpeeds = new double[4];
  double[] wheelSpeeds = new double[4];
  boolean reported;

  public MecaDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor,
      SpeedController rearRightMotor) {
    super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    speedControllers = new SpeedController[] { frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor };
  }

  @Override
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
    if (!reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_MecanumCartesian, 4);
      reported = true;
    }

    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
    ySpeed = applyDeadband(ySpeed, m_deadband);

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    // Compensate for gyro angle.
    Vector2d input = new Vector2d(ySpeed, xSpeed);
    input.rotate(-gyroAngle);

    inputSpeeds[MotorType.kFrontLeft.value] = input.x + input.y + zRotation;
    inputSpeeds[MotorType.kFrontRight.value] = -input.x + input.y - zRotation;
    inputSpeeds[MotorType.kRearLeft.value] = -input.x + input.y + zRotation;
    inputSpeeds[MotorType.kRearRight.value] = input.x + input.y - zRotation;

    normalize(wheelSpeeds);

    set(MotorType.kFrontLeft, m_maxOutput);
    set(MotorType.kFrontRight, -m_maxOutput);
    set(MotorType.kRearLeft, m_maxOutput);
    set(MotorType.kRearRight, -m_maxOutput);

    feed();
  }

  void calc(MotorType mt) {
    PIDController c = controllers[mt.value];
    double inputSpeed = inputSpeeds[mt.value];
    double measuredSpeed = encoders[mt.value].getRate();
    wheelSpeeds[mt.value] = c.calculate(measuredSpeed, inputSpeed);
  }

  void set(MotorType mt, double m) {
    // Treat new PID computed power output as an "adjustment"
    SpeedController sc = speedControllers[mt.value];
    double output = wheelSpeeds[mt.value] * m;
    double rateOutput = sc.get() + output;
    rateOutput = Math.min(1.0, rateOutput);
    rateOutput = Math.max(-1.0, rateOutput);
    sc.set(rateOutput);
  }
}
