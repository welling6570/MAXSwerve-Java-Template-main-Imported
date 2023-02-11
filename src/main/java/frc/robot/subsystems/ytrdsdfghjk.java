package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class ytrdsdfghjk {
    private final CANSparkMax intakeLeft = new CANSparkMax(IntakeConstants.kIntakeLeftMotor, MotorType.kBrushed);
    private final CANSparkMax intakeRight  = new CANSparkMax(IntakeConstants.kIntakeRightMotor, MotorType.kBrushed);
    private final DoubleSolenoid grabblerDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.kForwardChannel, IntakeConstants.kReverseChannel);

}
