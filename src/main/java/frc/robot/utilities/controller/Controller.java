package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
    /**
     * @return The amount of x on the left joystick.
     */
    double getLeftX();

    /**
     * @return The amount of y on the left joystick.
     */
    double getLeftY();

    /**
     * @return The amount of x on the right joystick.
     */
    double getRightX();

    /**
     * @return The amount of y on the right joystick.
     */
    double getRightY();

    /**
     * @Trigger Triggers when the bottom button is pressed.
     */
    Trigger a();

    /**
     * @Trigger Triggers when the right button is pressed.
     */
    Trigger b();

    /**
     * @Trigger Triggers when the up button is pressed.
     */
    Trigger y();

    /**
     * @Trigger Triggers when the left button is pressed.
     */
    Trigger x();

    /**
     * @Trigger Triggers a zero of the swerve on the right analog trigger.
     */
    Trigger zero();
}
