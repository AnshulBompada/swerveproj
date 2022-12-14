package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SwerveMod {
    double global_rotation;
    WPI_TalonFX m_speed;
    WPI_TalonFX m_rotation;
    WPI_CANCoder rot_encoder;
    PWMSparkMax sim_speed;
    PWMSparkMax sim_rotation;

    public SwerveMod(WPI_TalonFX speed, WPI_TalonFX rotation, int encoder_port) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = new WPI_CANCoder(encoder_port);
    }

    public SwerveMod(PWMSparkMax speed, PWMSparkMax rotation) {
      sim_speed = speed;
      sim_rotation = rotation;
    }


    public void control(double speed, double rotation) {
      turntoang(m_rotation, rotation * 180);
      m_speed.set(speed);
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
      global_rotation =+ rotationdegrees;*/
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

  public void turntoang(double rotationdegrees) {
    while(rotationdegrees < rot_encoder.getAbsolutePosition() % 180) {
      if(180 - rotationdegrees > 180)
        m_rotation.set(-1);
      m_rotation.set(0);
    }
    m_rotation.set(0);
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
}