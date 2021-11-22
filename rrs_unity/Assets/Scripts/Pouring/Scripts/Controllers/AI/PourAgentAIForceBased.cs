using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
public class PourAgentAIForceBased : Agent
{
    /** Gameobject variables **/
    private GameObject EnvironmentAgent; // Standalone  Agent which has all relevant game objects as children required for this task.
    public GameObject SensorHand; // Sensor hand where target is kept.
    public GameObject PouringAgent; // A child of Environment agent. This is where a source solid actor will be spawned 
                                    // (with liquid as child of spawned solid actor).
    public GameObject sources; // Sources to choose from.
    public GameObject sinks; // Targets to choose from.
    private Solid target = null; // Solid object as a child of Pouring agent where liquid is poured.
    private Liquid liquid = null; // Pouring agent's spawned source actor's child game object. (initially spawned contained in source)
    private Solid source = null; //  Solid object as a child of Pouring agent where liquid pouring takes place.
    public PhysicMaterial targetMaterial; // Physics material of the target.

    /**================================================================-**/

    /** Internal variabes **/
    private string sourceName; // Source actor's name.
    private string sinkName; // Target actor's name.
    private bool stoppingTrigger = false; // Boolean flag to signal source actor to stop. 
    private static float target_to_fill = 0.0f; // Flag to raise when target liquid level reached.
    private Dictionary<string, float> particleDictionary = null; // Dictionary to keep state of weight of particles in different containers.
    private float depthOfSourceContainer = 0.0f;
    private float approxDiameterOfSourceContainer = 0.0f;
    private float weightOfTarget = 0.0f;
    private float weightOfLiquid = 0.0f;
    private bool withinTargetRim;
    private float liquidParticleWeight = 0.0f;
    private float originalWeight = 0.0f;
    private float spillage = 0.0f;
    private float differenceFillLevel = 0.0f;
    private bool isActionDone = false;
    private int episodeNum = 0;
    /**================================================================**/

    /** NVIDIA Flex based variables **/
    public FlexContainer flexParticleContainer; // Flex container. Shared by all actors i.e. both solid and liquid.
    public FlexArrayAsset liquidAsset; // Flex liquid array asset. To be provided to the agent.
    /**================================================================**/

    /** ML-Agents variables **/
    EnvironmentParameters defaultAcademyParameters; // ML-Agent's environment parameters.

    Vector3 axisOfRotation = Vector3.zero; // Source's axis of rotation for pouring action.
    float sourceTurningSpeed = 0.0f; // Source rotation angle as suggested by agent's action.
    float landingMultiplier = 0.0f; // How far to land from the target container.
    /**================================================================**/


    /**==============All relevant co-routines below====================**/

    private IEnumerator waiter(int seconds)
    {
        yield return new WaitForSeconds(seconds);
    } 
    
    /**================================================================**/


    /** Methods to create solid and liquid objects in the scene **/

    /// <summary>
    /// Creates Solid Game Object. Based on string provided, chooses source or target.
    /// </summary>
    /// <param name="name"> Name of the soild actor to be generated.</param>
    private void createSolidObjcts(string name)
    {
        if (name == "SourceObject_" + EnvironmentAgent.gameObject.name)
        {
            createSourceObject(name);
        }
        else if (name == "TargetObject_" + EnvironmentAgent.gameObject.name)
        {
            createTargetObject(name);
        }
        else print(name + " not configured for flex solid actor..");
    }

    /// <summary>
    /// Creates source solid object and assigns kinematic properties to it.
    /// </summary>
    /// <param name="name">Name of the source object</param>
    private void createSourceObject(string name)
    {
        Transform[] children = sources.GetComponentsInChildren<Transform>();
        GameObject randomGO = (GameObject)((Transform)children[Random.Range(1, children.Length)]).gameObject;
        if ("sources" != randomGO.name)
        {
            if (source != null) {
                source.remove();
                Destroy(source.getSolidObject());
            }
            source = PouringAgent.AddComponent<Solid>();
            source.setName(name);
            Vector3 position = randomiseSourceLocations();
            Quaternion rotation = Quaternion.Euler(Vector3.zero);
            Vector3 scale = new Vector3(18.0f, 18.0f, 18.0f);
            source.setReferenceGO(randomGO);
            source.setPosition(position);
            source.setRotation(rotation);
            source.setLocalScale(scale);
            source.createSolid(setRigidBody: true, isKinematic: true, addCollider: true);
            depthOfSourceContainer = source.getObjectDepth();
            approxDiameterOfSourceContainer = source.getObjectDiameter();
        }
    }

    /// <summary>
    /// Creates target solid object and assigns kinematic properties to it.
    /// </summary>
    /// <param name="name">Name of the target</param>
    private void createTargetObject(string name)
    {
        Transform[] children = sinks.GetComponentsInChildren<Transform>();
        GameObject randomGO = (GameObject)((Transform)children[Random.Range(1, children.Length)]).gameObject;

        if ("targets" != randomGO.name)
        {
            if (target != null) {
                target.remove();
                Destroy(target.getSolidObject());
            }
            target = PouringAgent.AddComponent<Solid>();
            target.setName(name);
            Vector3 position = SensorHand.transform.position + new Vector3(0, SensorHand.transform.localScale.y, 0);
            Quaternion rotation = Quaternion.Euler(Vector3.zero);
            Vector3 scale = new Vector3(15.0f, 15.0f, 15.0f);
            target.setReferenceGO(randomGO);
            target.setPosition(position);
            target.setRotation(rotation);
            target.setLocalScale(scale);
            target.createSolid(setRigidBody: true, isKinematic: false, addCollider: true);
            // target.getSolidObject().GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezePositionX | RigidbodyConstraints.FreezePositionZ | 
            //                                                                 RigidbodyConstraints.FreezeRotation;
            target.setChildOf(SensorHand);
            target.getSolidObject().GetComponent<MeshCollider>().material = targetMaterial;
            weightOfTarget = target.getSolidObject().GetComponent<Rigidbody>().mass;
            SensorHand.GetComponent<ForceInformation>().target = target.getSolidObject();
        }
    }

    /// <summary>
    /// Randomises spawn location for source solid acor at the begining of each episode. 
    /// </summary>
    /// <returns> Spawn position from randomly sampled x, y and z coordinates. </returns>
    private Vector3 randomiseSourceLocations()
    {
        return new Vector3(Random.Range(-8.0f, 8.0f), Random.Range(7.0f, 9.0f), Random.Range(-8.0f, 8.0f)) + EnvironmentAgent.transform.position;
    }

    /// <summary>
    /// Create liquid object as a child of source object. This is a child of PouringAgent.
    /// </summary>
    private void createLiquid() {
        bool isReset = source == null;

        if (!isReset)
        {   
            if (liquid != null) {
                liquid.remove();
                Destroy(liquid.getLiquid());
            }
            liquid = PouringAgent.AddComponent<Liquid>();
            liquid.setFlexParticleContainer(flexParticleContainer);
            liquid.setLiquidAsset(liquidAsset);
            liquid.setLiquidHandler(null);
            liquid.setEnvironmentName(EnvironmentAgent.name);
            liquid.createLiquid(500f);
            ResetLiqParticleState();
            liquid.getLiquid().AddComponent<RigidBodyParticleSystemCollision>();
            SensorHand.GetComponent<ForceInformation>().targetWeight = target_to_fill;
            originalWeight = liquidAsset.maxParticles * liquidParticleWeight;
        }
    }

    /// <summary>
    /// Resets the liquid particle state dictionary.
    /// </summary>
    private void ResetLiqParticleState()
    {
        if (particleDictionary != null)
        {
            particleDictionary = null;
        }

        particleDictionary = new Dictionary<string, float>();

        particleDictionary.Add("numParticlesInSource", 0);
        particleDictionary.Add("numParticlesInTarget", 0);
        particleDictionary.Add("numParticlesPouring", 0);
        particleDictionary.Add("numParticlesSpilled", 0);
        particleDictionary.Add("transientVelocity", 0);
    }

    /**================================================================**/

    /** Scene related methods. Agent controlled by ML-Agents model     **/

    /// <summary>
    /// Called once when ML agents training start. Initializes academy parameters and starts the scene.
    /// </summary>
    public override void Initialize()
    {
        defaultAcademyParameters = Academy.Instance.EnvironmentParameters;
        EnvironmentAgent = this.gameObject;
        StartScene();
    }

    /// <summary>
    /// Called once. Initialises names, and resets the scene.
    /// </summary>
    private void StartScene()
    {
        sourceName = "SourceObject_" + EnvironmentAgent.gameObject.name;
        sinkName = "TargetObject_" + EnvironmentAgent.gameObject.name;
    }

    /// <summary>
    /// Resets the scene.
    /// </summary>
    private void ResetScene()
    {
        episodeNum += 1;
        isActionDone = false;
        axisOfRotation = Vector3.zero;
        sourceTurningSpeed = 0.0f;
        landingMultiplier = 0.0f;
        spillage = 0.0f;
        target_to_fill = Random.Range(0.15f, 0.45f);
        liquidParticleWeight = Random.Range(0.0005f, 0.001f);
        weightOfLiquid = 0.0f;
        differenceFillLevel = 0.0f;
        createSolidObjcts(sourceName);
        createSolidObjcts(sinkName);
        createLiquid();
        stoppingTrigger = false;
    }
    
    /// <summary>
    /// Tasks to do when episode begins.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        ResetScene();
    }

    
    /// <summary>
    /// Adds sensor observations used by PPO RL Agent to make actions.
    /// Overall 12 observations.
    /// </summary>
    /// <param name="sensor"> Vector sensor observations.</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        if (source != null)
        {
            sensor.AddObservation(depthOfSourceContainer); // 1 observation of the depth of the source object.
            sensor.AddObservation(approxDiameterOfSourceContainer); // 1 observation of approximated diameter of source object's mouth.
            sensor.AddObservation(target.getObjectDiameter()); // 1 observation for target's approximate diameter.
            
            sensor.AddObservation(Vector2.Distance(new Vector2(source.getSolidObject().transform.position.x,
                                                                source.getSolidObject().transform.position.z), 
                                                    new Vector2(target.getSolidObject().transform.position.x, 
                                                                target.getSolidObject().transform.position.z))); // 1 observations for planar distance.

            sensor.AddObservation(source.getSolidObject().transform.rotation.eulerAngles / 360.0f); // 3 observations for source's rotation.
            sensor.AddObservation(liquid.getFlexParticleContainer().viscosity / 150.0f); // 1 observation for normalised viscosity of fluid.
            sensor.AddObservation(originalWeight); // 1 observation for original weight of liquid in environment.
            sensor.AddObservation(particleDictionary["numParticlesInSource"] * liquidParticleWeight); // 1 observation for weight of liquid in the source container. 
            sensor.AddObservation(spillage / originalWeight); // 1 observation for the current level of the spillage as a ratio w.r.t original weight.
            sensor.AddObservation((differenceFillLevel / target_to_fill)); // 1 observation for the current difference in liquid weight as a proportion from desired.
        }
    }

    /// <summary>
    /// Tasks to do when an action is received from the PPO. Actions are continuous domain. 
    /// </summary>
    /// <param name="actions"> Actions received from PPO agent.</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        float similarity = Vector3.Dot(target.getSolidObject().transform.up, Vector3.up);
        // wait for target object to stablize on sensor. 
        if (similarity > 0.99999f)
        {
            performPouringAction(actions);
            AddReward(-1f / MaxStep);
        }
    }

    /**================================================================**/


    /**---------------Scene action logic methods below-----------------**/    
    private void performPouringAction(ActionBuffers actionBuffers)
    {
        int actionIndex = -1;
        float x = Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // move source by this value in x axis.
        float z = Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // move source by this value in z axis.
        sourceTurningSpeed = 50f * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // rotate source by this angle about axis of rotation defined by relative position between source and target.
        landingMultiplier = 1.2f * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // How far to land from target container.
        
        bool isReset = (source == null) || (target == null);
        if (!isReset)
        {
            if (source.getCollisionOccured()) {
                // Source object collided with target while moving. Penalize and restart
                float finalReward = -1.0f;
                SetReward(finalReward);
                print("Episode " + episodeNum + ": Collision event");
                print("Episode " + episodeNum + ": Final Reward: " + GetCumulativeReward());
                EndEpisode();
            }

            if (!stoppingTrigger)
            {
                if (landingMultiplier > 0)
                {
                    Vector3 sourcePosition = source.getSolidObject().transform.position;
                    Vector3 targetPosition = target.getSolidObject().transform.position;
                    MeshCollider targetMeshCollider = target.getSolidObject().GetComponent<MeshCollider>();
                    Bounds targetBounds = targetMeshCollider.bounds;
                    float targetWidth = target.getObjectDiameter();

                    withinTargetRim = sourcePosition.x >= targetBounds.min.x &&
                                                sourcePosition.x <= targetBounds.max.x &&
                                                sourcePosition.z >= targetBounds.min.z &&
                                                sourcePosition.z <= targetBounds.max.z;

                    float planarDistanceBefore = Vector2.Distance(new Vector2(sourcePosition.x, sourcePosition.z), new Vector2(targetPosition.x, targetPosition.z));
                    float planarDistanceAfter = Vector2.Distance(new Vector2(sourcePosition.x + x, sourcePosition.z + z), new Vector2(targetPosition.x, targetPosition.z));
                    if (withinTargetRim)
                    {
                        if (planarDistanceAfter > planarDistanceBefore)
                        {
                            sourcePosition += new Vector3(x, 0, z);
                            source.getSolidObject().transform.position = Vector3.MoveTowards(source.getSolidObject().transform.position, sourcePosition, 2.0f * Time.deltaTime);
                            AddReward(0.001f);
                            stoppingTrigger = planarDistanceAfter >= landingMultiplier * targetWidth;
                        }
                    }
                    else
                    {
                        if (planarDistanceAfter < planarDistanceBefore)
                        {
                            sourcePosition += new Vector3(x, 0, z);
                            source.getSolidObject().transform.position = Vector3.MoveTowards(source.getSolidObject().transform.position, sourcePosition, 2.0f * Time.deltaTime);
                            AddReward(0.001f);
                            stoppingTrigger = planarDistanceAfter <= landingMultiplier * targetWidth;
                        }
                    }
                }
            }
            else
            {
                particleDictionary = liquid.getLiquidState(source.getSolidObject(), target.getSolidObject(), particleDictionary);

                /// If using speed control by agent, axis-angle representation for rotations would be beneficial. Can use Rodrigue's rotation formula.

                // Agent controls source container's rotation about axis of rotation as defined by look at direction of target.
                Vector3 relativePositionOfTarget = (target.getSolidObject().transform.position - source.getSolidObject().transform.position).normalized;
                axisOfRotation = Vector3.Cross(Vector3.up, relativePositionOfTarget);
                
                weightOfLiquid = SensorHand.GetComponent<ForceInformation>().getMeasuredWeight();

                differenceFillLevel = Mathf.Abs(target_to_fill - weightOfLiquid);

                spillage = particleDictionary["numParticlesSpilled"] * liquidParticleWeight;
                    
                float filledLevel = (float)weightOfLiquid / target_to_fill;
                float similarity = Vector3.Dot(source.getSolidObject().transform.up, target.getSolidObject().transform.up);

                
                if (differenceFillLevel >= 0.02f && weightOfLiquid <= target_to_fill)
                {
                    if (sourceTurningSpeed > 0 && particleDictionary["numParticlesPouring"] == 0f)
                    {
                        source.getSolidObject().transform.Rotate(axisOfRotation, sourceTurningSpeed * Time.deltaTime, Space.Self);
                    }

                    if (sourceTurningSpeed < 0 && particleDictionary["numParticlesPouring"] > 10 && particleDictionary["transientVelocity"] > 1.0f)
                    {
                        source.getSolidObject().transform.Rotate(axisOfRotation, sourceTurningSpeed * Time.deltaTime, Space.Self);
                    }

                    if (particleDictionary["numParticlesInSource"] == 0)
                    {
                        isActionDone = true;
                    }

                    if (spillage > 0.20f) {
                        isActionDone = true;
                    }
                }
                else 
                {
                    if (sourceTurningSpeed < 0)
                    {
                        source.getSolidObject().transform.Rotate(axisOfRotation, sourceTurningSpeed * Time.deltaTime, Space.Self);
                    }

                    // Source object retracted atleast 97% to it's original orientation. Add final rewards and end episode.
                    if (similarity >= 0.75)
                    {
                        isActionDone = true;
                    }
                }
                
                if (isActionDone) {
                    if (differenceFillLevel < target_to_fill) 
                    {
                        if (spillage > 0.20f)
                        {
                            print("Episode " + episodeNum + ": Excessive spillage");
                            AddReward(-spillage / originalWeight); // Penalize all actions which result in spillages greater than 20%. (some spillage might be due to rendering liquid in small objects)
                        } 
                        else
                        {
                            AddReward(0.1f);
                        }
                        
                        // Reward if weight poured is within approximately +- 0.05 mass units. Penalise otherwise.
                        // finalReward = (differenceFillLevel < 0.06f ? 1.0f : -(differenceFillLevel / target_to_fill));
                        print("Episode " + episodeNum + ": Target: " + target_to_fill + " Filled: " + weightOfLiquid + " Difference: " + differenceFillLevel + " Spilled: " + spillage);
                        // SetReward(finalReward);
                        AddReward((differenceFillLevel < 0.06f ? 1.0f : -(differenceFillLevel / target_to_fill)));
                        print("Episode " + episodeNum + ": Final Reward: " + GetCumulativeReward());
                        EndEpisode();
                    } else {
                        print("Episode " + episodeNum + ": discarding current episode due to unstable measurements");
                        SetReward(0.0f);
                        EndEpisode();
                    }
                }

            }
        }
    }
    /**================================================================**/
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        if (Input.GetKey(KeyCode.D))
        {
            continuousActionsOut[0] = 0.2f;
        }
        if (Input.GetKey(KeyCode.W))
        {
            continuousActionsOut[1] = 0.2f;
        }
        if (Input.GetKey(KeyCode.A))
        {
            continuousActionsOut[0] = -0.2f;
        }
        if (Input.GetKey(KeyCode.S))
        {
            continuousActionsOut[1] = -0.2f;
        }
        
        float rotationAngle = 100f * Input.GetAxis("Horizontal");
        continuousActionsOut[2] = rotationAngle;
        
        if (Input.GetKey(KeyCode.None)) {
            continuousActionsOut[0] = 0.0f;
            continuousActionsOut[1] = 0.0f;
            continuousActionsOut[2] = 0.0f;
        }
    }
}
