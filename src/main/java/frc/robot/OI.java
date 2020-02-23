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
    public JoystickButton joyTrigger = new JoystickButton(joystick, 1);
    // The fist port on a controller is 1 :/
    public JoystickButton xboxA = new JoystickButton(xboxController, 1);
    public JoystickButton xboxB = new JoystickButton(xboxController, 2);

    public OI(){
        // Driver
        //lookAtBallButton.whenHeld(new LookAtBallCommand(), true);

        // Operator
        xboxA.whenPressed(new ShootCommand());
        xboxB.whenPressed(new IntakeCommand());
    }

    public double deadzone(double value){
        return Math.abs(value) < Constants.Deadzone ? 0 : value;
    }

    public double throttle(double d, boolean deadzone) {
        if(deadzone)
            d = deadzone(d);
        return throttle(d);
    }

    public double throttle(double d) {
        double speedMultiplier = (-joystick.getThrottle() + 1) / 2;
        return Constants.ManualThrottle * speedMultiplier * (Math.signum(d) * d * d);
    }
}
