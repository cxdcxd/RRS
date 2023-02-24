using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForceInformationLT : MonoBehaviour
{
    FixedJoint fixedJoint;

    public float targetWeight = 0.0f;
    public GameObject target;
    public GameObject workspace;
    private float measuredWeight;
    private float kgTog = 1000.0f;

    public float getMeasuredWeight()
    {
        this.weightMeasurement();
        return this.measuredWeight;
    }

    public void setMeasuredWeight(float measuredWeight)
    {
        this.measuredWeight = measuredWeight;
    }

    private void weightMeasurement() {
        float weight = 0.0f;
        fixedJoint = gameObject.GetComponent<FixedJoint>();
        float forceMagnitude = fixedJoint.currentForce.magnitude;
        weight = forceMagnitude / Physics.gravity.magnitude;
        float workspaceMass = this.workspace == null ? 0.0f : this.workspace.GetComponent<Rigidbody>().mass;
        weight = weight - ((target.GetComponent<Rigidbody>() != null ? target.GetComponent<Rigidbody>().mass : 0.0f) + 
                            workspaceMass);
        this.setMeasuredWeight(weight);
    }
    // Start is called before the first frame update
    void Start()
    {
        fixedJoint = gameObject.GetComponent<FixedJoint>();
    }

    // Update is called once per frame
    void Update()
    {
    }

    private void OnGUI() {
        if (target != null)
        {
            GUI.skin.label.fontSize = 20;

            if (fixedJoint.gameObject.name.Contains("sourceHand")) {
                GUI.Label(new Rect(10, 10, 500, 40), "Liquid in source: " + kgTog * this.getMeasuredWeight() + " g");
                
            } else
            {
                GUI.Label(new Rect(10, 50, 500, 40), "Target: " + kgTog*targetWeight + " g");
                GUI.Label(new Rect(10, 90, 500, 40), "Filled weight: " + kgTog*this.getMeasuredWeight() + " g");
            }
        }
    }
}
