 using UnityEngine;
 using System.Collections;
 using System.Collections.Generic;

 
 
 public class ParticleCollisionDetector : MonoBehaviour
 {
     public ParticleSystem part;
     public List<ParticleCollisionEvent> collisionEvents;
    
     public Vector3 force;

     public Vector3 force_sum = new Vector3(0,0,0);
    
    public Vector3 force_total = new Vector3(0,0,0);

    public Vector3 force_total_last = new Vector3(0,0,0);

    int cnt=0;
     public GameObject Scale;
     void Start()
     {
         part = GetComponent<ParticleSystem>();
         
         collisionEvents = new List<ParticleCollisionEvent>();
     }


    void OnParticleCollision(GameObject other)
    {

       

        if (other.name == "Scale")
        {
            //Debug.Log("THIS PARTICLE SYSTEM HIT:" + other.name);

            int numCollisionEvents = part.GetCollisionEvents(other, collisionEvents);

            Rigidbody rb = other.GetComponent<Rigidbody>();
            int i = 0;
            

            //while (i < numCollisionEvents)
            //{
            //    if (rb)
            //    {
            //        Vector3 pos = collisionEvents[i].intersection;


                    
            //    }
            //    i++;
            //}

            //force_total += force_sum;

            //if(cnt==15){
            //force_total = force_total_last.y>force_total.y? force_total_last:force_total;

            Scale.GetComponent<WeightScale>().UpdateWeightex(numCollisionEvents);

            //cnt=0;
            //force_total_last = force_total;
            ////force_total = new Vector3(0,0,0);
            //}
            //cnt++;

        }
    }
 }