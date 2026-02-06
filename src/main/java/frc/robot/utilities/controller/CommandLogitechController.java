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

    public double getR1Axis() {
        // FIXME: this might not be the right button number i just want to fulfill the
        // implementation
        return controller.getRawAxis(4);
    }

    public Trigger R1() {
        return new Trigger(() -> this.getR1Axis() > 0.4);
    }

    @Override
    public Trigger zero() {
        return this.R1();
    }
}
