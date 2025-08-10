using UnityEngine;

public class HUDControlEvent : GameEvent
{
  public string hudId;
  public bool shouldShow;
  public string direction;

  public HUDControlEvent(string hudId, bool shouldShow, string direction = "")
  {
    this.hudId = hudId;
    this.shouldShow = shouldShow;
    this.direction = direction;
  }
}

public class LEDControlEvent : GameEvent
{
    public float blinkSpeed;
    public float timerSpeed;
    public bool shouldBlink;
    public float volume;
    public string direction;
    public SoundType soundType; 
    public Color ledColor;    

    public LEDControlEvent(float blinkSpeed, float timerSpeed, bool shouldBlink, float volume)
    {
        this.blinkSpeed = blinkSpeed;
        this.timerSpeed = timerSpeed;
        this.shouldBlink = shouldBlink;
        this.volume = volume;
        this.direction = direction;
    }
    
    public LEDControlEvent(float blinkSpeed, float timerSpeed, bool shouldBlink, float volume, string direction, SoundType soundType)
    {
        this.blinkSpeed = blinkSpeed;
        this.timerSpeed = timerSpeed;
        this.shouldBlink = shouldBlink;
        this.volume = volume;
        this.direction = direction;
        this.soundType = soundType;
        this.ledColor = Color.clear;
    }
}
