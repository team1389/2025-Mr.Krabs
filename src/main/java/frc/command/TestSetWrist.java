package frc.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorArm;

public class TestSetWrist extends Command {
    public ElevatorArm elevatorArm;
    double targetAngle;
    private final PIDController pidController;


    public TestSetWrist(ElevatorArm elevatorArm, double targetAngle) {
        this.elevatorArm = elevatorArm;
        this.targetAngle = targetAngle;
        double pValue = 3;
        double iValue = 0.0;
        double dValue = 0.0;
        pidController = new PIDController(pValue, iValue, dValue);
        pidController.setTolerance(.005);
        pidController.setSetpoint(targetAngle);
        SmartDashboard.putNumber("WristAbsEncoder", elevatorArm.getWristEncoder());
    }

    @Override
    public void execute() {
        double currentAngle = elevatorArm.getWristEncoder();
        double power = pidController.calculate(currentAngle);

        // Limit the power to prevent damage to the mechanism.  These values
        // should be based on your hardware.
        power = -Math.max(-.3, Math.min(.3, power)); // Example: Limit between -1 and 1

        elevatorArm.setManualWrist(power);

        SmartDashboard.putNumber("WristAbsEncoder", currentAngle);
        SmartDashboard.putNumber("WristPower", power); // Put power on dashboard
        SmartDashboard.putNumber("WristError", pidController.getPositionError()); // Put error on dashboard
    }

    @Override
    public void end(boolean interrupted) {
        elevatorArm.setManualWrist(0);
    }
}
//911 <-- Call for pizza