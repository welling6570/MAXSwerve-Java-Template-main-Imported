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
    private final CANSparkMax intakeLeft = new CANSparkMax(IntakeConstants.kIntakeLeftMotor, MotorType.kBrushless);
    private final CANSparkMax intakeRight  = new CANSparkMax(IntakeConstants.kIntakeRightMotor, MotorType.kBrushless);
    private final DoubleSolenoid grabblerDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.kForwardChannel, IntakeConstants.kReverseChannel);

    
    public void jeremyRennerVibeCheck(double out, double in){
        intakeLeft.set((in - out)*.1); 
        intakeRight.set(-(in - out)*.1);
    }
    
    public void jeremyRennerHug() {
        grabblerDoubleSolenoid.set(kReverse);
    }
    public void jeremyRennerRelease() {
        grabblerDoubleSolenoid.set(kForward);
    }
}
