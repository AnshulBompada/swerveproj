package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class SwerveMod {
    private WPI_TalonFX m_speed;
    private WPI_TalonFX m_rotation;
    private WPI_CANCoder rot_encoder;
    private CANCoderConfiguration config;
    private PIDController pid;
    private StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT;

    public SwerveMod(WPI_TalonFX speed, WPI_TalonFX rotation, int encoder_port) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = new WPI_CANCoder(m_rotation.getDeviceID());
        config = new CANCoderConfiguration();
        config.sensorCoefficient = 360 / 4096.0;
        config.unitString = "deg";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        rot_encoder.configAllSettings(config);

        DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration();
        
        m_speed.configFactoryDefault();
        m_speed.setInverted(TalonFXInvertType.CounterClockwise);
        m_speed.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
        m_speed.config_kP(0, 0.044057);
        m_speed.config_kF(0, 0.028998);

        rot_encoder.configFactoryDefault();
        rot_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        rot_encoder.setPositionToAbsolute();
        rot_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        m_rotation.configFactoryDefault();
        m_rotation.setInverted(TalonFXInvertType.CounterClockwise);
        m_rotation.configRemoteFeedbackFilter(rot_encoder, 0);
        m_rotation.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        m_rotation.setSelectedSensorPosition(rot_encoder.getAbsolutePosition());
        m_rotation.config_kP(0, 0.2);
        m_rotation.config_kD(0, 0.01);

        SmartDashboard.putNumber("Wheel degrees", m_rotation.getSelectedSensorPosition());

        pid = new PIDController(0.2, 0, 0.01);
    }

    public void control(double speed, double rotation) {
      SmartDashboard.updateValues();
      accturntoang(rotation * 180);
      m_speed.set(speed);
    }

    public void accturntoang(double rotationdegrees) {
//      double error = m_rotation.getSelectedSensorPosition() - convert(rotationdegrees);
//      double other_error = convert(rotationdegrees) - m_rotation.getSelectedSensorPosition();
      /*if(Math.abs(error) > 2048){
        m_rotation.set(-pid.calculate(m_rotation.getSelectedSensorPosition(), m_rotation.getSelectedSensorPosition() - other_error));
      }
      if(Math.abs(error) < 2048){
        m_rotation.set(pid.calculate(m_rotation.getSelectedSensorPosition(), convert(rotationdegrees)));
      }*/

      m_rotation.set(ControlMode.Position, convert(rotationdegrees));
    }

    public double convert(double degrees) {
      double ticks = degrees * 4096/180;
      return ticks;
    }

    public double getwheeldegs() {
        return convert(m_rotation.getSelectedSensorPosition());
    }












    /*
    public void turntoang(double rotationdegrees) {
      double r = 10;
      double error;
      while(rotationdegrees - r < rot_encoder.getAbsolutePosition() || rotationdegrees + r > rot_encoder.getAbsolutePosition()) {
        error = rotationdegrees - rot_encoder.getAbsolutePosition();
        if(Math.abs(error) > 180) {
          m_rotation.set(-1);
        }
        if(Math.abs(error) < 180) {
          m_rotation.set(1);
        }
        m_rotation.set(0);
      }
    }

    public void dumbturntoang(double rotationdegrees) {
      double range = 0.2;
      double error;
      while(rotationdegrees - range < rot_encoder.getAbsolutePosition() || rotationdegrees + range > rot_encoder.getAbsolutePosition()) {
        error = rot_encoder.getAbsolutePosition() - rotationdegrees;
        if(Math.abs(error) > 180) {
          m_rotation.set(-dumbcalc(error));
        }
        if(Math.abs(error) < 180) {
          m_rotation.set(dumbcalc(error));
        }
      }
    }

    public double dumbcalc(double degrees) {
      if(Math.abs(degrees) > 180) {
        return (2/Math.PI) * Math.atan(Math.toRadians(Math.abs(degrees) - 180));
      }
      return (2/Math.PI) * Math.atan(Math.toRadians(degrees));
    }



  
    public void turntime(WPI_TalonFX motor, double speed, double time){
      Timer timer = new Timer();
      timer.start();
      while(timer.get() < time) {
        motor.set(speed);
  }
      motor.set(0);
  }

  public void turntime(PWMSparkMax motor, double speed, double time){
    Timer timer = new Timer();
    timer.start();
    while(timer.get() < time) {
      motor.set(speed);
  }
    motor.set(0);
  }

  public void setposition(double pos) {
    rot_encoder.setPosition(pos);
  }
  
  public void simcontrol(double speed, double rotation) {
    turntoang(sim_rotation, rotation * 180);
    sim_speed.set(speed);
  }

  public void turntoang(WPI_TalonFX motor, double rotationdegrees) {
//      double time = (75 / (13*180))/rotationdegrees - (75 / (13*180))* global_rotation;
//      double speed =1;
/*      if(time < 0) {
      time = -time;
      speed = -speed;
    }
    turntime(motor, speed, time);
    global_rotation =+ rotationdegrees;
  }

  public void turntoang(PWMSparkMax motor, double rotationdegrees) {
  double time = (75 / (13*180))/rotationdegrees - (75 / (13*180))* global_rotation;
  double speed =1;
  if(time < 0) {
    time = -time;
    speed = -speed;
  }
  turntime(motor, speed, time);
  global_rotation =+ rotationdegrees;
  }
*/
}