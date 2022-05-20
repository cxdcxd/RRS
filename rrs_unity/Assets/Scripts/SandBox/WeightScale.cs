using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WeightScale : MonoBehaviour
{
    // Start is called before the first frame update
    float forceToMass;
 
    public float combinedForce_0;
    public float combinedForce_1;

    public float calculatedMass;

    public static float massDisplay;
 
    public int registeredRigidbodies;

    Dictionary<Rigidbody, float> impulsePerRigidBody = new Dictionary<Rigidbody, float>();
 
    float currentDeltaTime;
    float lastDeltaTime;

    
    private void Awake()
    {
        combinedForce_1 = 0;
        forceToMass = 1f / Physics.gravity.magnitude;
    }
    
   
    public void UpdateWeight()
    {
        registeredRigidbodies = impulsePerRigidBody.Count;
        combinedForce_0 = 0;
        
 
        foreach (var force in impulsePerRigidBody.Values)
        {
            combinedForce_0 += force;

        }

      

      
    }
    
    public void UpdateWeightex(float force)
    {

        combinedForce_1 = force * 0.5f;
      
    }


    private void FixedUpdate()
    {
        lastDeltaTime = currentDeltaTime;
        currentDeltaTime = Time.deltaTime;
        
    }
    
    private void OnCollisionStay(Collision collision)
    {
        if (collision.rigidbody != null)
        {
            if (impulsePerRigidBody.ContainsKey(collision.rigidbody))
                impulsePerRigidBody[collision.rigidbody] = collision.impulse.y / lastDeltaTime;
            else
                impulsePerRigidBody.Add(collision.rigidbody, collision.impulse.y / lastDeltaTime);
 
            UpdateWeight();
        }
    }
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.rigidbody != null)
        {
            if (impulsePerRigidBody.ContainsKey(collision.rigidbody))
                impulsePerRigidBody[collision.rigidbody] = collision.impulse.y / lastDeltaTime;
            else
                impulsePerRigidBody.Add(collision.rigidbody, collision.impulse.y / lastDeltaTime);
 
            UpdateWeight();
        }
    }
    private void OnCollisionExit(Collision collision)
    {
        if (collision.rigidbody != null)
        {
            impulsePerRigidBody.Remove(collision.rigidbody);
            UpdateWeight();
        }
    }

}
