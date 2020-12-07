﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetMQ;

public class ColorCamera : MonoBehaviour
{
    public float fps = 20f;
    public int jpeg_quality = 50;
    public int width = 800;
    public int height = 600;

    public delegate void DelegateDataChanged(byte[] buffer);
    public event DelegateDataChanged delegateCameraDataChanged;

    private Texture2D screenShot;
    private RenderTexture rt;
    private Camera cam;
    private bool inited = false;
    private byte[] buffer = new byte[1];
    private float timer = 0.0f;

    void Start()
    {
        cam = GetComponent<Camera>();
        screenShot = new Texture2D(width, height, TextureFormat.RGB24, false);
        rt = new RenderTexture(width, height, 24);

        if (cam != null && screenShot != null && rt != null)
            inited = true;
        else
            print("Error in camera");
    }

    void FixedUpdate()
    {
        timer += Time.deltaTime;
        if (timer > (1 / fps) && inited)
        {
            cam.targetTexture = rt;
            cam.Render();
            RenderTexture.active = rt;
            screenShot.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            cam.targetTexture = null;
            RenderTexture.active = null; // Added to avoid errors

            //Decode camera
            buffer = screenShot.EncodeToJPG(jpeg_quality);

            //Update data changed event
            delegateCameraDataChanged?.Invoke(buffer);

            //print("Color Changed");

            timer = 0;
        }
    }
}
