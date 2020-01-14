/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;

/**
 * Add your docs here.
 */
public class OI {
    public XboxController xboxController = new XboxController(0);
    public Joystick joystick = new Joystick(1);
    public JoystickButton aButton = new JoystickButton(xboxController, 1);
    public JoystickButton bButton = new JoystickButton(xboxController, 2);

    public OI(){
        aButton.whenPressed(new ShootCommand());
        bButton.whenPressed(new IntakeCommand());
    }

    public static double deadzone(double value){
        return Math.abs(value) < Constants.Deadzone ? 0 : value;
    }
}
