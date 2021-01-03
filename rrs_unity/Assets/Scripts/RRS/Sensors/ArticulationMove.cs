using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArticulationMove : MonoBehaviour
{
    private ArticulationBody root;

    public ArticulationBody[] joints;

    public bool enable = true;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        foreach (ArticulationBody item in joints)
        {

            item.enabled = enable;
        }
    }
}
