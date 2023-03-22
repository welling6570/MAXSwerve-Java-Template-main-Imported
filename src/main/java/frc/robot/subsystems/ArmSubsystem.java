package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private static final SparkMaxRelativeEncoder.Type kRelEncType = SparkMaxRelativeEncoder.Type.kHallSensor;
    private static final int kCPR = 8192;
    //private RelativeEncoder m_alternateEncoder;
    
    private CANSparkMax extensionSparkMax;
    
    private SparkMaxPIDController m_extensionPidController;
    
    private RelativeEncoder m_extensionEncoder;    

    private final WPI_TalonFX JeremyRenner;

public ArmSubsystem() {

    extensionSparkMax = new CANSparkMax(ArmConstants.kExtensionMotor, MotorType.kBrushless);
    JeremyRenner = new WPI_TalonFX(ArmConstants.kAngleMotor); 

    extensionSparkMax.restoreFactoryDefaults();
    extensionSparkMax.setIdleMode(IdleMode.kBrake);

    JeremyRenner.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
    ArmConstants.kPIDLoopIdx,
    ArmConstants.kTimeoutMs);
    //m_alternateEncoder = extensionMotor.getAlternateEncoder(kAltEncType, kCPR); 
    JeremyRenner.configForwardSoftLimitEnable(true);
    JeremyRenner.configReverseSoftLimitEnable(true);
    JeremyRenner.configForwardSoftLimitThreshold(ArmConstants.JeremyRennerforwardThreshold);
    JeremyRenner.configReverseSoftLimitThreshold(ArmConstants.JeremyRennerreverseThreshold);
    JeremyRenner.configPeakOutputForward(0.4);
    JeremyRenner.configPeakOutputReverse(-0.4);
    JeremyRenner.config_kP(0, 1.0);
    JeremyRenner.config_kI(0, 0.0);
    JeremyRenner.config_kD(0, 0.0);
    JeremyRenner.configAllowableClosedloopError(0, 500);
    //This here sets a motion profile so that our arm wobbles less.
   
   
    JeremyRenner.configMotionAcceleration(5000);
    //changed sensorUnitsPer100msPerSec from 30000 to 40000 after match 31
    // changed from 40000 to 20000 and then down to 5000 3/7/23
    JeremyRenner.configMotionCruiseVelocity(60000);
    JeremyRenner.configMotionSCurveStrength(0);
    //JeremyRenner.configMotion

       
    m_extensionEncoder = extensionSparkMax.getEncoder();
    m_extensionEncoder.setPositionConversionFactor(1);
    //m_extensionEncoder = extensionSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    
    extensionSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    extensionSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extensionSparkMax.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.extensionMotorforwardThreshhold);
    extensionSparkMax.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.extensionMotorreverseThreshhold);
    extensionSparkMax.setSmartCurrentLimit(40, 20);
    //m_extensionEncoder.setPosition(50);
    //m_extensionEncoder.setInverted(true);
    
    m_extensionPidController = extensionSparkMax.getPIDController();    
    m_extensionPidController.setOutputRange(-0.5, 0.5);
    m_extensionPidController.setFeedbackDevice(m_extensionEncoder);   
    //m_extensionPidController.
    // set PID coefficients
    m_extensionPidController.setP(ArmConstants.kExtensionP);
    m_extensionPidController.setI(ArmConstants.kExtensionI);
    m_extensionPidController.setD(ArmConstants.kExtensionD);
    m_extensionPidController.setIZone(ArmConstants.kExtensionIz);
    m_extensionPidController.setFF(ArmConstants.kExtensionFF);
    m_extensionPidController.setOutputRange(ArmConstants.kExtensionMinOutput, ArmConstants.kExtensionMaxOutput);
    extensionSparkMax.burnFlash();

  

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
        if (JeremyRenner.getSelectedSensorPosition()>-50000) {
            extensionSparkMax.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.extensionMotorSafeThreshold);
        } else {
            extensionSparkMax.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.extensionMotorreverseThreshhold);
        }
        if (m_extensionEncoder.getPosition()<-60) {
            JeremyRenner.configForwardSoftLimitThreshold(ArmConstants.JeremyRennerSafeThreshold);
        } else {
            JeremyRenner.configForwardSoftLimitThreshold(ArmConstants.JeremyRennerforwardThreshold);
         }
    }

    public void Angxtend(double reach, double lift){  
        //SmartDashboard.putNumber("Angle: ", JeremyRenner.getSelectedSensorPosition());
        JeremyRenner.set(TalonFXControlMode.MotionMagic, lift);
        m_extensionPidController.setReference(reach,  CANSparkMax.ControlType.kPosition);
    }

    public void AlterLift(){  
        JeremyRenner.set(TalonFXControlMode.MotionMagic, (JeremyRenner.getSelectedSensorPosition()-5000 ));
    }

    public boolean getReach(){
        if (m_extensionEncoder.getPosition() <= ArmConstants.getReachCondition) {
            return true;
        } else {
            return false;
        }
    }
    public boolean reachReturned(){
        if (m_extensionEncoder.getPosition() >= ArmConstants.reachReturnedCondition) {
            return true;
        } else {
            return false;
        }
    }
    public boolean getLift(){
        if (JeremyRenner.getSelectedSensorPosition() <= ArmConstants.getLiftCondition) {
            return true; } else {return false;}
        //changed getSelectedSensorPostition from <=-120000 to -130000 after match 31
    }

    public boolean getLift2(){
        if (JeremyRenner.getSelectedSensorPosition() <= -1000) {
            return true; } else {return false;}
    }
}