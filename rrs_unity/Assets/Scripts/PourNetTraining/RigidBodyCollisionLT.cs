using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidBodyCollisionLT : MonoBehaviour {
    public SolidLT solid;
    public RobotLT robot;
    private void OnCollisionEnter(Collision other) {
        if(this.gameObject.name.Contains("mug") || this.gameObject.name.Contains("cup") || 
            this.gameObject.name.Contains("glass")) {
            if (other.gameObject.name.Contains("bowl") || other.gameObject.name.Contains("targetHand")) {
                solid.setCollisionOccured(true);
            }
        }
        
        if(this.gameObject.name.Contains("sourceHand")) {
            if (other.gameObject.name.Contains("targetHand") || other.gameObject.name.Contains("Target")) {
                robot.setCollision(true);
            }
        }
    }
}