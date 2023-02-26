package frc.robot.Commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class IntakeManipulation extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

  public IntakeManipulation(IntakeSubsystem intakesubsystem) {
    m_intakeSubsystem = intakesubsystem;
    addRequirements(m_intakeSubsystem);
  }

  public void initialize() {
    m_intakeSubsystem.jeremyRennerRelease();    
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

