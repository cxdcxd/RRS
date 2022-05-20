using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class reader : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        FixedJoint a = GetComponent<FixedJoint>();
        float b = a.currentForce.x * a.currentForce.x + a.currentForce.y * a.currentForce.y + a.currentForce.z * a.currentForce.z;
        float c = Mathf.Sqrt(b);

        print("Weight " + c);
    }
}
