using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForceInformation : MonoBehaviour
{
    FixedJoint fixedJoint;

    public float targetWeight = 0.0f;
    public GameObject target;
    public GameObject workspace;
    private float measuredWeight;

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
        weight = weight - (target.GetComponent<Rigidbody>().mass + workspaceMass);
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
            if (fixedJoint.gameObject.name.Contains("sourceHand")) {
                GUI.Label(new Rect(10, 10, 1000, 20), "Liquid in source: " + this.getMeasuredWeight());
            } else
            {
                GUI.Label(new Rect(10, 40, 1000, 20), "Target: " + targetWeight);
                GUI.Label(new Rect(10, 80, 1000, 20), "Filled weight: " + this.getMeasuredWeight());
            }
        }
    }
}
