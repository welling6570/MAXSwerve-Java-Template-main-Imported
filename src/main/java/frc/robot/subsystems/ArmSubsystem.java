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
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private static final SparkMaxRelativeEncoder.Type kRelEncType = SparkMaxRelativeEncoder.Type.kQuadrature;
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
    JeremyRenner.configForwardSoftLimitThreshold(-1000);
    JeremyRenner.configReverseSoftLimitThreshold(-185000);
    JeremyRenner.configPeakOutputForward(0.4);
    JeremyRenner.configPeakOutputReverse(-0.4);
    JeremyRenner.config_kP(0, 1.0);
    JeremyRenner.config_kI(0, 0.0);
    JeremyRenner.config_kD(0, 0.0);
    JeremyRenner.configAllowableClosedloopError(0, 500);
    //This here sets a motion profile so that our arm wobbles less.
    JeremyRenner.configMotionAcceleration(30000);
    JeremyRenner.configMotionCruiseVelocity(60000);

    m_extensionEncoder = extensionMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    m_extensionEncoder.setPositionConversionFactor(2.35);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extensionMotor.setSoftLimit(SoftLimitDirection.kForward, 81);
    extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, 50);
    m_extensionEncoder.setPosition(50);
    m_extensionEncoder.setInverted(true);
    
    m_extensionPidController = extensionMotor.getPIDController();    

    m_extensionPidController.setFeedbackDevice(m_extensionEncoder);   
    //m_extensionPidController.
    // set PID coefficients
    m_extensionPidController.setP(ArmConstants.kExtensionP);
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
    public void getPositions() {
        SmartDashboard.putNumber("Extension", m_extensionEncoder.getPosition());
        SmartDashboard.putNumber("Angle", JeremyRenner.getSelectedSensorPosition());
        if (JeremyRenner.getSelectedSensorPosition()>-40000) {
            extensionMotor.setSoftLimit(SoftLimitDirection.kForward, 50);
        } else {
            extensionMotor.setSoftLimit(SoftLimitDirection.kForward, 81);
        }
        if (m_extensionEncoder.getPosition()>51) {
            JeremyRenner.configForwardSoftLimitThreshold(-60000);
        } else {
            JeremyRenner.configForwardSoftLimitThreshold(-1000);
         }
    }


    public void Angxtend(double reach, double lift){  
        SmartDashboard.putNumber("Angle: ", JeremyRenner.getSelectedSensorPosition());
        JeremyRenner.set(TalonFXControlMode.MotionMagic, lift);
        m_extensionPidController.setReference(reach,  CANSparkMax.ControlType.kPosition);
    }

    public void AlterLift(){  
        JeremyRenner.set(TalonFXControlMode.MotionMagic, (JeremyRenner.getSelectedSensorPosition()-5000 ));
    }
}