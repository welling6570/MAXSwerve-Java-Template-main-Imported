package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeLeft = new CANSparkMax(IntakeConstants.kIntakeLeftMotor, MotorType.kBrushed);
    private final CANSparkMax intakeRight  = new CANSparkMax(IntakeConstants.kIntakeRightMotor, MotorType.kBrushed);
    private final DoubleSolenoid grabblerDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.kForwardChannel, IntakeConstants.kReverseChannel);

    
    public void jeremyRennerVibeCheck(double out, double in){
        intakeLeft.set((in - out)); 
        intakeRight.set(-(in - out));
    }
    
    public void donstretch() {
        grabblerDoubleSolenoid.set(kForward);
    }
    public void donfold() {
        grabblerDoubleSolenoid.set(kReverse);
    }
}
