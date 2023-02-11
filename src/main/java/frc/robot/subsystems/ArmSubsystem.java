package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotor, MotorType.kBrushed);
    public final CANSparkMax angleMotor  = new CANSparkMax(ArmConstants.kAngleMotor, MotorType.kBrushed);
    public SparkMaxPIDController m_ExtensionPidController;
    public double kExtensionP, kExtensionI, kExtensionD, kExtensionIz, kExtensionFF, kExtensionMaxOutput, kExtensionMinOutput;
   
    public SparkMaxPIDController m_AnglePidController;
    public double kAngleP, kAngleI, kAngleD, kAngleIz, kAngleFF, kAngleMaxOutput, kAngleMinOutput;

    public SparkMaxAlternateEncoder m_extensionEncoder;
    public SparkMaxAlternateEncoder m_angleEncoder;

    m_extensionEncoder = extensionMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    m_angleEncoder = angleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);

    m_extensionPidController = extensionMotor.getPIDController();
    m_anglePidController = angleMotor.getPIDController();

    m_extensionPidController.setFeedbackDevice(m_extensionEncoder);
    m_anglePidController.setFeedbackDevice(m_angleEncoder);

    // set PID coefficients
    //m_extensionPidController.setP(kExtensionP);
    m_extensionPidController.setI(kExtensionI);
    m_extensionPidController.setD(kExtensionD);
    m_extensionPidController.setIZone(kExtensionIz);
    m_extensionPidController.setFF(kExtensionFF);
    m_extensionPidController.setOutputRange(kExtensionMinOutput, kExtensionMaxOutput);

    m_anglePidController.setP(kAngleP);
    m_anglePidController.setI(kAngleI);
    m_anglePidController.setD(kAngleD);
    m_anglePidController.setIZone(kAngleIz);
    m_anglePidController.setFF(kAngleFF);
    m_anglePidController.setOutputRange(kAngleMinOutput, kAngleMaxOutput);

    
}}