using UnityEngine;
using System.Collections;

namespace RRS
{
    public class FPS : MonoBehaviour
    {
        float deltaTime = 0.0f;
        string text = "";
   

        void Update()
        {
            deltaTime += (Time.deltaTime - deltaTime) * 0.1f;
            float msec = deltaTime * 1000.0f;
            float fps = 1.0f / deltaTime;
            text = string.Format("FPS: {0:0.0} ms ({1:0.} fps)", msec, fps);
        }

        void OnGUI()
        {
            GUI.contentColor = Color.black;
            GUI.Label(new Rect(10, 10, 100, 200), text);
        }

    }
}
