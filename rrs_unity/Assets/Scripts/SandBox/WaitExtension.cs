using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;


public static class WaitExtension
{
    public static void Wait(this MonoBehaviour mono,float delay,UnityAction action){
        mono.StartCoroutine(ExecuteAction(delay,action));
    }

    private static IEnumerator ExecuteAction(float delay,UnityAction action){
        yield return new WaitForSecondsRealtime(delay);
        action.Invoke();
        yield break;
    }
}
