package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
    //private RelativeEncoder m_alternateEncoder;
    
    private CANSparkMax extensionMotor;
    
    private SparkMaxPIDController m_extensionPidController;
    
    private RelativeEncoder m_extensionEncoder;    

    private final WPI_TalonFX JeremyRenner;

public ArmSubsystem() {

    extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotor, MotorType.kBrushed);
    JeremyRenner = new WPI_TalonFX(ArmConstants.kAngleMotor); 

    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(IdleMode.kCoast);

    JeremyRenner.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
    ArmConstants.kPIDLoopIdx,
    ArmConstants.kTimeoutMs);
    //m_alternateEncoder = extensionMotor.getAlternateEncoder(kAltEncType, kCPR); 
    JeremyRenner.configForwardSoftLimitEnable(true);
    JeremyRenner.configReverseSoftLimitEnable(true);
    JeremyRenner.configForwardSoftLimitThreshold(10000);
    JeremyRenner.configReverseSoftLimitThreshold(-10000);
    JeremyRenner.configPeakOutputForward(0.2);
    JeremyRenner.configPeakOutputReverse(-0.2);
    JeremyRenner.config_kP(0, 1.0);
    JeremyRenner.config_kI(0, 0.0);
    JeremyRenner.config_kD(0, 0.0);
    JeremyRenner.configAllowableClosedloopError(0, 5000);
    m_extensionEncoder = extensionMotor.getAlternateEncoder(kAltEncType, 4096);
    m_extensionEncoder.setPositionConversionFactor(2.35);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extensionMotor.setSoftLimit(SoftLimitDirection.kForward, 2);
    extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, -2);
    m_extensionPidController = extensionMotor.getPIDController();    

    m_extensionPidController.setFeedbackDevice(m_extensionEncoder);   

    // set PID coefficients
    //m_extensionPidController.setP(kExtensionP);
    m_extensionPidController.setI(ArmConstants.kExtensionI);
    m_extensionPidController.setD(ArmConstants.kExtensionD);
    m_extensionPidController.setIZone(ArmConstants.kExtensionIz);
    m_extensionPidController.setFF(ArmConstants.kExtensionFF);
    m_extensionPidController.setOutputRange(ArmConstants.kExtensionMinOutput, ArmConstants.kExtensionMaxOutput);
    extensionMotor.burnFlash();

  

}
    public void extendopatronum(double reach) {
        if(JeremyRenner.getSelectedSensorPosition()<300){
            m_extensionPidController.setReference(0,  CANSparkMax.ControlType.kPosition);
        }else{
             m_extensionPidController.setReference(reach,  CANSparkMax.ControlType.kPosition);
        }
       


    }

    public void wingardiumleviosa(double lift) {

        JeremyRenner.set(TalonFXControlMode.Position, lift);
        

    }
    public void Angxtend(double reach, double lift){  
        SmartDashboard.putNumber("Angle: ", JeremyRenner.getSelectedSensorPosition());
        if (JeremyRenner.getSelectedSensorPosition()<0) {
            extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        } else {
            extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, -2);
        }
        if (m_extensionEncoder.getPosition()>1) {
            JeremyRenner.configForwardSoftLimitThreshold(5000);
        } else {
            JeremyRenner.configForwardSoftLimitThreshold(10000);
        }
        JeremyRenner.set(TalonFXControlMode.Position, lift);
        m_extensionPidController.setReference(reach,  CANSparkMax.ControlType.kPosition);
            }    
    }