using UnityEngine;

public class SoundEvent : GameEvent
{
  public enum Direction
  {
    Ahead, AheadRight, Right, BehindRight,
    Behind, BehindLeft, Left, AheadLeft
  }

  public Direction direction;
  public float distance;
  public float volume;
  public SoundType soundType;
  public Vector3 sourcePosition;
  public Vector3 listenerPosition;
  public float blinkThreshold;

  public SoundEvent(Direction direction, float distance, float volume, SoundType soundType, Vector3 sourcePosition, Vector3 listenerPosition, float blinkThreshold)
  {
    this.direction = direction;
    this.distance = distance;
    this.volume = volume;
    this.soundType = soundType;
    this.sourcePosition = sourcePosition;
    this.listenerPosition = listenerPosition;
    this.blinkThreshold = blinkThreshold;
  }
}
