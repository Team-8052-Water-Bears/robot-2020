/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Postions of the wheel relative to the physical center of robot
    // These values are assumptions and not tested
    public static final Translation2d lfWheel = new Translation2d(0, 1);
    public static final Translation2d rfWheel = new Translation2d(1, 1);
    public static final Translation2d lbWheel = new Translation2d(0, 0);
    public static final Translation2d rbWheel = new Translation2d(1, 0);

    // This shoudl probably be moved
    public static final MecanumDriveKinematics MecanumKinematics = new MecanumDriveKinematics(lfWheel, rfWheel, lbWheel, rbWheel);
    public static final double Deadzone = 0.1;
    public static final boolean GyroReversed = false;
	public static final String VisonTableKey = "Vision";


    public static class AutoConstants{
        public static final double MaxWheelVelocity = 1;
		public static double MaxVelocity = 1; // m/s
        public static double MaxAcceleration = 1; // m/s^2
    }
}
