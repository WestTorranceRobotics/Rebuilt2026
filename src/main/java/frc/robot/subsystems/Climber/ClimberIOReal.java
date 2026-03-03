package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIOReal extends SubsystemBase implements ClimberIO {

    @Override
    public Distance getCurrentPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCurrentPosition'");
    }

    @Override
    public LinearVelocity getCurrentVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCurrentVelocity'");
    }

    @Override
    public Voltage getVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
    }

    @Override
    public void setTargetPosition(Distance distance) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetPosition'");
    }

    @Override
    public void stopClimber() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopClimber'");
    }
}
