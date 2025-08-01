using UnityEngine;

[System.Serializable]
public class PIDController
{
  [Header("PID Gains")]
  public float kp = 1.0f; // Proportional gain
  public float ki = 0.1f; // Integral gain  
  public float kd = 0.5f; // Derivative gain

  [Header("Limits")]
  public float integralLimit = 10f; // 적분 제한값
  public float outputLimit = 1f; // 출력 제한값

  private float previousError = 0f; // 이전 오차값
  private float integral = 0f; // 적분 누적값
  private float lastTime = 0f;

  public PIDController(float kp, float ki, float kd)
  {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.lastTime = Time.time;
  }

  public float Calculate(float setpoint, float currentValue, float deltaTime)
  {
    float error = setpoint - currentValue;

    // 적분항 계산
    integral += error * deltaTime;
    integral = Mathf.Clamp(integral, -integralLimit, integralLimit);

    // 미분항 계산
    float derivative = deltaTime > 0 ? (error - previousError) / deltaTime : 0f;

    // PID output
    float output = kp * error + ki * integral + kd * derivative;
    output = Mathf.Clamp(output, -outputLimit, outputLimit);

    previousError = error;

    return output;
  }

  public void Reset()
  {
    previousError = 0f;
    integral = 0f;
    lastTime = Time.time;
  }

  public void SetGains(float newKp, float newKi, float newKd)
  {
    kp = newKp;
    ki = newKi;
    kd = newKd;
  }
}
