package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandLogitechController extends CommandGenericHID  {
    private GenericHID controller;

    public CommandLogitechController(int port) {
        super(port);
        controller = super.getHID();
    }

    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    public double getLeftY() {
        return -controller.getRawAxis(1);
    }

    public double getRightX() {
        return controller.getRawAxis(4);
    }

    public double getRightY() {
        return -controller.getRawAxis(5);
    }

    public double getRightAnalogTrigger() {
        return controller.getRawAxis(3);
    }

    public Trigger a() {
        return new Trigger(() -> controller.getRawButton(1));
    }

    public Trigger b() {
        return new Trigger(() -> controller.getRawButton(2));
    }

    public Trigger y() {
        return new Trigger(() -> controller.getRawButton(4));
    }

    public Trigger x() {
        return new Trigger(() -> controller.getRawButton(3));
    }
}