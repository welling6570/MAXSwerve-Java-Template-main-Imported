package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotor, MotorType.kBrushed);
   
    //private m_extensionEncoder = new SparkMaxAlternateEncoder(extensionMotor,4096);

    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
    private RelativeEncoder m_alternateEncoder;
    
    private final WPI_TalonFX JeremyRenner = new WPI_TalonFX(ArmConstants.kAngleMotor); 
    
    private SparkMaxPIDController m_extensionPidController;
    
    private SparkMaxAlternateEncoder m_extensionEncoder;    

public ArmSubsystem() {

    JeremyRenner.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
    ArmConstants.kPIDLoopIdx,
    ArmConstants.kTimeoutMs);
    m_alternateEncoder = extensionMotor.getAlternateEncoder(kAltEncType, kCPR); 

    m_extensionPidController = extensionMotor.getPIDController();    

    m_extensionPidController.setFeedbackDevice(m_extensionEncoder);   

    // set PID coefficients
    //m_extensionPidController.setP(kExtensionP);
    m_extensionPidController.setI(ArmConstants.kExtensionI);
    m_extensionPidController.setD(ArmConstants.kExtensionD);
    m_extensionPidController.setIZone(ArmConstants.kExtensionIz);
    m_extensionPidController.setFF(ArmConstants.kExtensionFF);
    m_extensionPidController.setOutputRange(ArmConstants.kExtensionMinOutput, ArmConstants.kExtensionMaxOutput);

  

}
    public void extendopatronum(double reach) {
        m_extensionPidController.setReference(reach,  CANSparkMax.ControlType.kPosition);


    }

    public void wingardiumleviosa(double reach) {

        JeremyRenner.set(TalonFXControlMode.Position, reach);
        

    }
}