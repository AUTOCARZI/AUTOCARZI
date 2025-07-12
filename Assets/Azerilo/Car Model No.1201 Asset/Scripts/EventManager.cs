using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public static class EventManager
{
    private static Dictionary<Type, UnityEventBase> eventDictionary = new Dictionary<Type, UnityEventBase>();

    public static void Subscribe<T>(UnityAction<T> listener) where T : GameEvent
    {
        Type eventType = typeof(T);

        if (!eventDictionary.ContainsKey(eventType))
        {
            eventDictionary[eventType] = new UnityEvent<T>();
        }

        ((UnityEvent<T>)eventDictionary[eventType]).AddListener(listener);
    }

    public static void Unsubscribe<T>(UnityAction<T> listener) where T : GameEvent
    {
        Type eventType = typeof(T);

        if (eventDictionary.ContainsKey(eventType))
        {
            ((UnityEvent<T>)eventDictionary[eventType]).RemoveListener(listener);
        }
    }

    public static void Publish<T>(T gameEvent) where T : GameEvent
    {
        Type eventType = typeof(T);

        if (eventDictionary.ContainsKey(eventType))
        {
            ((UnityEvent<T>)eventDictionary[eventType]).Invoke(gameEvent);
        }
    }

    public static void Clear()
    {
        eventDictionary.Clear();
    }
}

public abstract class GameEvent { }

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

    public SoundEvent(Direction dir, float dist, float vol, SoundType soundType, Vector3 sourcePos, Vector3 listenerPos, float threshold)
    {
        direction = dir;
        distance = dist;
        volume = vol;
        soundType = soundType;
        sourcePosition = sourcePos;
        listenerPosition = listenerPos;
        blinkThreshold = threshold;
    }
}

public class CarInputEvent : GameEvent
{
    public float throttle;
    public float steer;
    public bool brake;
    public bool headlights;

    public CarInputEvent(float th, float st, bool br, bool lights)
    {
        throttle = th;
        steer = st;
        brake = br;
        headlights = lights;
    }
}

public class HUDControlEvent : GameEvent
{
    public string hudId;
    public bool shouldShow;
    public string direction;

    public HUDControlEvent(string id, bool show, string direction = "")
    {
        hudId = id;
        shouldShow = show;
        direction = direction;
    }
}

public class LEDControlEvent : GameEvent
{
    public float blinkSpeed;
    public float timerSpeed;
    public bool shouldBlink;
    public float volume;
    public string direction;

    public LEDControlEvent(float bSpeed, float tSpeed, bool blink, float vol, string dir)
    {
        blinkSpeed = bSpeed;
        timerSpeed = tSpeed;
        shouldBlink = blink;
        volume = vol;
        direction = dir;
    }
}

public class MovementControlEvent : GameEvent
{
    public float throttle;
    public float steer;
    public bool brake;

    public MovementControlEvent(float th, float st, bool br)
    {
        throttle = th;
        steer = st;
        brake = br;
    }
}