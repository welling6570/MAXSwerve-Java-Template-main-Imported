package frc.robot.Commands;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmManipulation extends CommandBase {
  // The subsystem the command runs on
  private final ArmSubsystem m_armSubsystem;
  
  public ArmManipulation(ArmSubsystem armsubsystem) {
    m_armSubsystem = armsubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {
    m_armSubsystem.Angxtend(50, -176000);
   // if (m_armSubsystem.getLift()>=-174000){
  //    m_armSubsystem.Angxtend(68.5, -176000);
    //}
  }

  @Override
  public boolean isFinished() {
   // if (m_armSubsystem.getReach()>=68 && m_armSubsystem.getLift()>=-174000){
   //   return true;
   // } else {
      return false;
  //  }
  }
}

