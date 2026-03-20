package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandLogitechController extends CommandGenericHID implements Controller {
    private GenericHID controller;

    public CommandLogitechController(int port) {
        super(port);
        controller = super.getHID();
    }

    @Override
    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    @Override
    public double getLeftY() {
        return -controller.getRawAxis(1);
    }

    @Override
    public double getRightX() {
        return controller.getRawAxis(4);
    }

    @Override
    public double getRightY() {
        return -controller.getRawAxis(5);
    }

    public double getRightAnalogTrigger() {
        return controller.getRawAxis(3);
    }

    public Trigger aOrCross() {
        return new Trigger(() -> controller.getRawButton(1));
    }

    public Trigger bOrCircle() {
        return new Trigger(() -> controller.getRawButton(2));
    }

    public Trigger yOrTriangle() {
        return new Trigger(() -> controller.getRawButton(4));
    }

    public Trigger xOrSquare() {
        return new Trigger(() -> controller.getRawButton(3));
    }

    public Trigger dPadLeft() {
        return new Trigger(() -> (controller.getPOV() == 270));
    }

    public Trigger dPadUp() {
        return new Trigger(() -> (controller.getPOV() == 0));
    }

    public Trigger dPadRight() {
        return new Trigger(() -> (controller.getPOV() == 90));
    }

    public Trigger dPadDown() {
        return new Trigger(() -> (controller.getPOV() == 180));
    }

    public Trigger R1() {
        return new Trigger(() -> controller.getRawButton(6));
    }

    @Override
    public Trigger zero() {
        return this.R1();
    }
}
