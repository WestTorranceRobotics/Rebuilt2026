package frc.robot.utilities.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;

public class LogitechController extends CommandGenericHID implements Controller {
    private GenericHID controller;

    public LogitechController(int port) {
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
        return new Trigger(() -> {
            Optional<Rotation2d> pov = controller.getPOV().getAngle();

            return pov.isPresent() ? pov.get().getDegrees() == 270.0 : false;
        });
    }

    public Trigger dPadUp() {
        return new Trigger(() -> {
            Optional<Rotation2d> pov = controller.getPOV().getAngle();

            return pov.isPresent() ? pov.get().getDegrees() == 0.0 : false;
        });
    }

    public Trigger dPadRight() {
        return new Trigger(() -> {
            Optional<Rotation2d> pov = controller.getPOV().getAngle();

            return pov.isPresent() ? pov.get().getDegrees() == 90.0 : false;
        });
    }

    public Trigger dPadDown() {
        return new Trigger(() -> {
            Optional<Rotation2d> pov = controller.getPOV().getAngle();

            return pov.isPresent() ? pov.get().getDegrees() == 180.0 : false;
        });
    }

    public Trigger R1() {
        return new Trigger(() -> controller.getRawButton(6));
    }

    @Override
    public Trigger zero() {
        return this.R1();
    }
}
