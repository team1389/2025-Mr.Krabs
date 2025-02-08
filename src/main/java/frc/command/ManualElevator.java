// package frc.command;

// import java.util.function.Supplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.subsystems.ElevatorSubsystem;

// public class ManualElevator extends Command{
//     ElevatorSubsystem elevatorSubsystem;
//     Supplier<Double> elevator, arm1;
//     Supplier<Boolean> arm2Up, arm2Down;

//     public ManualElevator(ElevatorSubsystem elevatorSubsystem, Supplier<Double> elevator, Supplier<Double> arm1, Supplier<Boolean> arm2Up, Supplier<Boolean> arm2Down){
//         this.elevatorSubsystem = elevatorSubsystem;
//         this.elevator = elevator;
//         this.arm1 = arm1;
//         this.arm2Up = arm2Up;
//         this.arm2Down = arm2Down;
//     }

//     @Override
//     public void execute(){
//         elevatorSubsystem.setManualElevator(MathUtil.clamp(elevator.get(), -1, 1));
//         elevatorSubsystem.setManualArm1(MathUtil.clamp(arm1.get(), -1, 1));
//         if(arm2Up.get()){
//             elevatorSubsystem.setManualArm2(.3);
//         } else if(arm2Down.get()){
//             elevatorSubsystem.setManualArm2(-.3);
//         } else if(!arm2Up.get() && !arm2Down.get()){
//             elevatorSubsystem.setManualArm2(0);
//         }
//     }


// }
