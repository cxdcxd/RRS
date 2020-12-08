using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Helper 
{
    public static Vector3 Ros2Unity(this Vector3 vector3)
    {
        return new Vector3(-vector3.y, vector3.z, vector3.x);
    }

    public static Vector3 Unity2Ros(this Vector3 vector3)
    {
        return new Vector3(vector3.z, -vector3.x, vector3.y);
    }

    public static Vector3 Ros2UnityScale(this Vector3 vector3)
    {
        return new Vector3(vector3.y, vector3.z, vector3.x);
    }

    public static Vector3 Unity2RosScale(this Vector3 vector3)
    {
        return new Vector3(vector3.z, vector3.x, vector3.y);
    }

    public static Quaternion Ros2Unity(this Quaternion quaternion)
    {
        return new Quaternion(quaternion.y, -quaternion.z, -quaternion.x, quaternion.w);
    }

    public static Quaternion Unity2Ros(this Quaternion quaternion)
    {
        return new Quaternion(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
    }

    public static float convertRotation(float input)
    {
        if (input < 0)
            input += 360;
        if (input >= 360)
            input -= 360;

        input = input - 90;
        input = 360 - input;

        if (input >= 360)
            input = input - 360;
        if (input < 0)
            input = input + 360;

        if (input > 180)
            input = input - 360; //[-180 , 0 , 180]
        
        return input;
    }

}
