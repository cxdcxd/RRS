/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;
using System;

public class TalkerExample : MonoBehaviourRosNode
{
    public Camera ImageCamera;
    public string FrameId = "Camera";
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    [Range(0, 100)]
    public int qualityLevel = 50;


    private Texture2D texture2D;
    private Rect rect;

    public string NodeName = "talker";
    public string Topic = "chatter";

    public float PublishingFrequency = 1.0f;

    protected override string nodeName { get { return NodeName; } }
    private Publisher<std_msgs.msg.String> chatterPublisher;
    private Publisher<sensor_msgs.msg.CompressedImage> chatterPublisher2;
    private std_msgs.msg.String msg;
    private sensor_msgs.msg.CompressedImage msg2;
    private int counter;

    private void UpdateImage(Camera _camera)
    {
        if (texture2D != null && _camera == this.ImageCamera)
            UpdateMessage();
    }

    public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);

    private void UpdateMessage()
    {
        TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
        double msecs = timeSpan.TotalMilliseconds;
        uint sec = (uint)(msecs / 1000);
        uint nsec = (uint)((msecs / 1000 - sec) * 1e+9);

        msg2.UpdateHeaderTime((int)sec, nsec);
        texture2D.ReadPixels(rect, 0, 0);

        msg2.Data = texture2D.EncodeToJPG(qualityLevel);
        msg2.Header.Frame_id = "cool";
        msg2.Format = "jpeg";

        //string json = JsonConvert.SerializeObject(message);
        //byte[] size = System.Text.Encoding.ASCII.GetBytes(json);
        UnityEngine.Debug.Log(" ROS2 sent");


        chatterPublisher2.Publish(msg2);
    }

    private void InitializeGameObject()
    {
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        Camera.onPostRender += UpdateImage;
    }

    protected override void StartRos()
    {
        InitializeGameObject();
        chatterPublisher = node.CreatePublisher<std_msgs.msg.String>(Topic);
        chatterPublisher2 = node.CreatePublisher<sensor_msgs.msg.CompressedImage>("chatter_image/compressed");
        msg = new std_msgs.msg.String();
        msg2 = new sensor_msgs.msg.CompressedImage();
        StartCoroutine("PublishMessage");
    }

    private void Start() {
        counter = 0;
    }

    IEnumerator PublishMessage()
    {
        for (;;)
        {
            msg.Data = "Hello World" + counter.ToString();
            Debug.Log("Publishing: " + counter.ToString());
            chatterPublisher.Publish(msg);
            counter++;
            yield return new WaitForSeconds(1.0f/PublishingFrequency);
        }
    }
}