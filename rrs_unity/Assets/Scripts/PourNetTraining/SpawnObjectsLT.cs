using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

public class SpawnObjectsLT: MonoBehaviour {
    
    /// <summary>
    /// Creates source solid object and assigns kinematic properties to it.
    /// </summary>
    /// <param name="name">Name of the source object</param>
    public static SolidLT createLiquidContainer(SolidLT solidObject, GameObject agent, GameObject references, string name, 
                                              PhysicMaterial solidObjectPhysicsMaterial, GameObject referred, bool randomizePositions=false, bool isKinematic=false)
    {
        if (solidObject != null) {
            solidObject.remove();
            Destroy(solidObject.getSolidObject());
            Destroy(solidObject);
        }

        //Transform[] children = references.GetComponentsInChildren<Transform>();

        //int randomChildIndex = Random.Range(0, references.transform.childCount);

        //GameObject randomGO = references.transform.GetChild(randomChildIndex).gameObject;

        GameObject randomGO = null;
        if (referred == null)
        {
            Transform[] children = references.GetComponentsInChildren<Transform>();
            int randomChildIndex = Random.Range(0, references.transform.childCount);
            randomGO = references.transform.GetChild(randomChildIndex).gameObject;
        }
        else
        {
            randomGO = referred;
        }

        solidObject = agent.AddComponent<SolidLT>();
        solidObject.setName(randomGO.gameObject.name);

        Vector3 position = agent.transform.position;
        if (randomizePositions)
            position = SpawnObjectsLT.randomiseSourceLocations()  + agent.transform.position;

        solidObject.setReferenceGO(randomGO);
        solidObject.setPosition(position);
        solidObject.setRotation(randomGO.transform.rotation);
        solidObject.setLocalScale(randomGO.transform.localScale);
        if (isKinematic)
            solidObject.createSolid(setRigidBody: false, isKinematic: isKinematic, addCollider: true);
        else
            solidObject.createSolid(setRigidBody: true, isKinematic: isKinematic, addCollider: true);
        solidObject.getSolidObject().GetComponent<MeshCollider>().material = solidObjectPhysicsMaterial;
        return solidObject;
    }

    /// <summary>
    /// Create liquid object as a child of source object. This is a child of PouringAgent.
    /// </summary>
    public static LiquidLT createLiquid(LiquidLT liquid, RobotLT liquidHandler, GameObject agent, FlexContainer flexParticleContainer, FlexArrayAsset liquidAsset, float liquidVolume,
                                     float density) {
        if (liquid != null) {
            liquid.remove();
            Destroy(liquid.getLiquid());
            Destroy(liquid);
        }
        liquid = agent.AddComponent<LiquidLT>();
        liquid.setFlexParticleContainer(flexParticleContainer);
        liquid.setLiquidAsset(liquidAsset);
        liquid.setLiquidHandler(liquidHandler);
        liquid.setEnvironmentName(agent.name);
        liquid.createLiquid(liquidVolume, density);

        return liquid;
    }

    public static RobotLT createRoboticArms(RobotLT robot, GameObject agent, GameObject roboPrefab, string name, PhysicMaterial material) {
        if (robot != null) {
            robot.remove();
            Destroy(robot.getRobotHand());
            Destroy(robot);
        }

        robot = agent.AddComponent<RobotLT>();
        robot.createRobot(roboPrefab, material, false);
        robot.getRobotHand().name = name;
        
        robot.getRobotHand().SetActive(true);
        return robot;
    }

    /// <summary>
    /// Randomises spawn location for source solid acor at the begining of each episode. 
    /// </summary>
    /// <returns> Spawn position from randomly sampled x, y and z coordinates. </returns>
    private static Vector3 randomiseSourceLocations()
    {
        return new Vector3(Random.Range(-0.5f, 0.5f), Random.Range(12.0f, 12.0f), Random.Range(-0.5f, 0.5f));
    }
}