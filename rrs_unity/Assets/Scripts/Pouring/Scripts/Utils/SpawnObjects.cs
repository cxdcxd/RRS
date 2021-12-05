using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

public class SpawnObjects: MonoBehaviour {
    
    /// <summary>
    /// Creates source solid object and assigns kinematic properties to it.
    /// </summary>
    /// <param name="name">Name of the source object</param>
    public static Solid createLiquidContainer(Solid solidObject, GameObject agent, GameObject references, string name, 
                                              PhysicMaterial solidObjectPhysicsMaterial, bool randomizePositions=false)
    {
        if (solidObject != null) {
            solidObject.remove();
            Destroy(solidObject.getSolidObject());
            Destroy(solidObject);
        }

        Transform[] children = references.GetComponentsInChildren<Transform>();

        int randomChildIndex = Random.Range(0, references.transform.childCount);

        GameObject randomGO = references.transform.GetChild(randomChildIndex).gameObject;

        solidObject = agent.AddComponent<Solid>();
        solidObject.setName(randomGO.gameObject.name);

        Vector3 position = agent.transform.position;
        if (randomizePositions)
            position = SpawnObjects.randomiseSourceLocations()  + agent.transform.position;

        solidObject.setReferenceGO(randomGO);
        solidObject.setPosition(position);
        solidObject.setRotation(randomGO.transform.rotation);
        solidObject.setLocalScale(randomGO.transform.localScale);
        solidObject.createSolid(setRigidBody: false, isKinematic: false, addCollider: true);
        solidObject.getSolidObject().GetComponent<MeshCollider>().material = solidObjectPhysicsMaterial;
        return solidObject;
    }

    /// <summary>
    /// Create liquid object as a child of source object. This is a child of PouringAgent.
    /// </summary>
    public static Liquid createLiquid(Liquid liquid, Robot liquidHandler, GameObject agent, FlexContainer flexParticleContainer, FlexArrayAsset liquidAsset, float liquidVolume,
                                     float density) {
        if (liquid != null) {
            liquid.remove();
            Destroy(liquid.getLiquid());
            Destroy(liquid);
        }
        liquid = agent.AddComponent<Liquid>();
        liquid.setFlexParticleContainer(flexParticleContainer);
        liquid.setLiquidAsset(liquidAsset);
        liquid.setLiquidHandler(liquidHandler);
        liquid.setEnvironmentName(agent.name);
        liquid.createLiquid(liquidVolume, density);

        return liquid;
    }

    /// <summary>
    /// Create liquid object as a child of source object. This is a child of PouringAgent.
    /// </summary>
    public static Liquid createLiquid(Liquid liquid, Solid pourer, GameObject agent,
                                      FlexContainer flexParticleContainer, 
                                      FlexArrayAsset liquidAsset,
                                      float liquidVolume,
                                      float density)
    {
        if (liquid != null)
        {
            liquid.remove();
            Destroy(liquid.getLiquid());
            Destroy(liquid);
        }
        liquid = agent.AddComponent<Liquid>();
        liquid.setFlexParticleContainer(flexParticleContainer);
        liquid.setLiquidAsset(liquidAsset);
        liquid.setEnvironmentName(agent.name);
        liquid.createLiquid(liquidVolume, density, liquid_pourer:pourer);

        return liquid;
    }

    public static Robot createRoboticArms(Robot robot, GameObject agent, GameObject roboPrefab, string name, PhysicMaterial material) {
        if (robot != null) {
            robot.remove();
            Destroy(robot.getRobotHand());
            Destroy(robot);
        }

        robot = agent.AddComponent<Robot>();
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