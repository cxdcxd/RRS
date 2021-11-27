using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDController : MonoBehaviour
{
    // Start is called before the first frame update
    public float desire = 0;
    public float current = 0;
    public int axis = 0;
    public float p_g = 0.001f;
    void Start()
    {
        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (axis == 1)
        {
            //current = transform.localRotation.eulerAngles.x;
            //if ( current )
            //float e = (desire - current) * p_g;
            //if (e > 5) e = 5;
            //if (e < -5) e = -5;
            //transform.Rotate(new Vector3(0, e, 0));
        }
        if (axis == 2)
        {
            current = transform.localRotation.eulerAngles.y - 180;
            float e = current + (desire - current) * p_g;
            transform.localRotation = Quaternion.Euler(transform.localRotation.eulerAngles.x, e - 180, transform.localRotation.eulerAngles.z);
        }
        if (axis == 3)
        {
            current = transform.localRotation.eulerAngles.z - 180;
            float e = current + (desire - current) * p_g;
            transform.localRotation = Quaternion.Euler(transform.localRotation.eulerAngles.x, transform.localRotation.eulerAngles.y, e - 180);
        }



    }
}
