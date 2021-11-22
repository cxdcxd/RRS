using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidBodyCollision : MonoBehaviour {
    public Solid solid;
    public Robot robot;
    private void OnCollisionEnter(Collision other) {
        if(this.gameObject.name.Contains("Source")) {
            if (other.gameObject.name.Contains("Target") || other.gameObject.name.Contains("targetHand")) {
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