using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

public class roboPouring : MonoBehaviour
{
    /// <summary>
    /// The objects below are to be provided in runtime.
    /// </summary>
    public GameObject robot; // Robot game object with robot parts as assets.

    public GameObject sources; // Set of source objects to hold liquid for pouring.

    public GameObject targets; // Set of target objects in which liquid is poured.

    public FlexContainer flexParticleContainer; // Flex container as a slover for all flex objects.

    public FlexArrayAsset liquidAsset; // FLex array asset for liquid object.

    public PhysicMaterial solidObjectPhysicsMaterial; // Physics material to use with source and target containers.
    public PhysicMaterial robotPhysicsMaterial; // Physics material to use with robotic hand.

    /** ===================11======================================================================================== **/

    /// <summary>
    /// The objects below are private and used internally during the runtime
    /// </summary>

    private Solid source;
    private Solid target;
    private Liquid liquid;
    private Robot robotHandForSource;
    private Robot robotHandForTarget;

    /// <summary>
    /// The methods below are used to spawn environment actors in the scene
    /// </summary>

    /// <summary>
    /// Creates Solid Game Object. Based on string provided, chooses source or target.
    /// </summary>
    /// <param name="name"> Name of the soild actor to be generated.</param>
    private void createSolidObjcts(string name)
    {
        if (name == "SourceObject_" + gameObject.name)
        {
            source = SpawnObjects.createLiquidContainer(source, this.gameObject, sources, name, solidObjectPhysicsMaterial, randomizePositions: true);
        }
        else if (name == "TargetObject_" + gameObject.name)
        {
            target = SpawnObjects.createLiquidContainer(target, this.gameObject, targets, name, solidObjectPhysicsMaterial, randomizePositions: false);
        }
        else print(name + " not configured for flex solid actor..");
    }

    // Start is called before the first frame update
    Dictionary<string, float> particleDictionary = new Dictionary<string, float>();

    void Start()
    {
        robotHandForSource = SpawnObjects.createRoboticArms(robotHandForSource, this.gameObject, robot, "sourceHand", robotPhysicsMaterial);
        robotHandForTarget = SpawnObjects.createRoboticArms(robotHandForTarget, this.gameObject, robot, "targetHand", robotPhysicsMaterial);

        createSolidObjcts("TargetObject_" + gameObject.name);
        createSolidObjcts("SourceObject_" + gameObject.name);
        
        robotHandForTarget.holdObject(target, null, true, true, true);
        robotHandForSource.holdObject(source, null, true, true);
        
        // liquid = SpawnObjects.createLiquid(liquid, robotHandForSource, this.gameObject, flexParticleContainer, liquidAsset, 500, 1.0f);
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        // particleDictionary = liquid.getLiquidState(source.getSolidObject(), target.getSolidObject(), particleDictionary);
    }
}