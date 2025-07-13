using System;
using System.Collections.Generic;
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
