public class CarInputEvent : GameEvent
{
  public float throttle;
  public float steer;
  public bool brake;
  public bool headlights;

  public CarInputEvent(float throttle, float steer, bool brake, bool headlights)
  {
    this.throttle = throttle;
    this.steer = steer;
    this.brake = brake;
    this.headlights = headlights;
  }
}

public class MovementControlEvent : GameEvent
{
  public float throttle;
  public float steer;
  public bool brake;

  public MovementControlEvent(float throttle, float steer, bool brake)
  {
    this.throttle = throttle;
    this.steer = steer;
    this.brake = brake;
  }
}

public class LaneWarningEvent : GameEvent
{
    public float laneOffset;
    public float confidence;

    public LaneWarningEvent(float offset, float conf)
    {
        laneOffset = offset;
        confidence = conf;
    }
}

