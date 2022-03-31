using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

/**
Pouring agent based on PPO trained agent. 
**/
public class PourAgentAI : Agent
{

    public GameObject EnvironmentAgent; // Agent which has all relevant game objects as children. 
    public GameObject Workspace; // Workspace object
    private string sourceName; // Source actor's name.
    private string sinkName; // Target actor's name.
    public GameObject sources; // Sources to choose from.
    public GameObject sinks; // Targets to choose from.
    public FlexContainer flexParticleContainer; // Flex container. Shared by all actors i.e. both solid and liquid.
    public FlexArrayAsset liquidAsset; // Flex liquid array asset. To be provided to the agent.
    public GameObject PouringAgent; // A child of Environment agent. This is where a source solid actor will be spawned (with liquid as child of spawned solid actor).
    private GameObject targetActor; // Environment Agent independent gameobject for target. 
    private FlexArrayActor liquidActor; // Flex array actor for liquid.
    private GameObject liquid; // Pouring agent's spawned source actor's child game object. (initially spawned contained in source)
    private ParticleSystem liquidPS; // Particle system with liquid.
    private FlexParticleController liquidPC; // Flex particle controller for liquid.
    private ParticleSystem.Particle[] m_particles; // Array to keep each particle in liquid particle system.
    private Dictionary<string, int> particleDictionary = null; // Dictionary to keep state of number of particles in different containers.
    private GameObject sourceActor; // Spawned gameobject for source solid actor. Child of PouringAgent.
    EnvironmentParameters defaultAcademyParameters; // ML-Agent's environment parameters.
    private bool collisionWithObjects = false; //  Boolean flag for collision of source with any other rigid body.
    private bool stoppingTrigger = false; // Boolean flag to signal source actor to stop. 
    private float timer = 0.0f; // Timer.
    private float desiredRadius; // Desired length from target's position deemed good for source container to land into.
    private int target_to_fill; // Desired filling level in the target container.
    private bool target_filled; // Flag to raise when target liquid level reached.
    private float turnAroundLevel; // Level to reverse rotation. 
    private float fractionToFill; // Fraction of liquid particles to transfer from source to target.

    /// <summary>
    /// Randomize liquid parameters on scene reset. Parameters used are:
    /// 1. Viscosity (as depicted by color of liquid rendered from green to blue)
    /// </summary>
    private void CreateLiquidParticleContainer() {
        float viscosity = Random.Range(0f, 150f);
        flexParticleContainer.viscosity = viscosity;
        float color = viscosity / 150.0f;
        flexParticleContainer.fluidMaterial.color = Color.Lerp(Color.green, Color.blue, color);
    }

    /// <summary>
    /// Creates Flex Solid Actor. Based on string provided, chooses source or target.
    /// </summary>
    /// <param name="name"> Name of the soild actor to be generated.</param>
    private void createSolidActors(string name)
    {
        if (name == "SourceObject_" + EnvironmentAgent.gameObject.name)
        {
            createSourceActor(name);
        }
        else if (name == "TargetObject_" + EnvironmentAgent.gameObject.name)
        {
            createTargetActor(name);
        }
        else print(name + " not configured for flex solid actor..");
    }

    /// <summary>
    /// Creates the Flex solid actor for source container.
    /// </summary>
    /// <param name="name">Unique source container name</param>
    private void createSourceActor(string name)
    {
        if (sourceActor != null)
        {
            // Destroy already existing actor.
            Destroy(sourceActor);
        }

        Transform[] children = sources.GetComponentsInChildren<Transform>();
        GameObject randomGO = (GameObject)((Transform)children[Random.Range(1, children.Length)]).gameObject;

        if ("sources" != randomGO.name)
        {
            //spawn object
            sourceActor = new GameObject(name);
            sourceActor.transform.parent = PouringAgent.transform;

            //Add Components
            sourceActor.AddComponent<MeshFilter>();
            sourceActor.AddComponent<MeshCollider>();
            MeshRenderer mr = sourceActor.AddComponent<MeshRenderer>();
            sourceActor.transform.position = randomiseSourceLocations();
            sourceActor.transform.rotation = Quaternion.Euler(Vector3.zero);
            sourceActor.transform.localScale = new Vector3(18.0f, 18.0f, 18.0f);
            mr.sharedMaterial = randomGO.GetComponent<MeshRenderer>().material;
            sourceActor.GetComponent<MeshFilter>().sharedMesh = randomGO.GetComponent<MeshFilter>().mesh;
            sourceActor.GetComponent<MeshCollider>().sharedMesh = randomGO.GetComponent<MeshFilter>().mesh;
            sourceActor.AddComponent<Rigidbody>();
            sourceActor.GetComponent<Rigidbody>().isKinematic = true;
        }
    }

    /// <summary>
    /// Creates the target solid actor.
    /// </summary>
    /// <param name="name">Unique name for target.</param>
    private void createTargetActor(string name)
    {
        if (targetActor != null)
        {
            // Destroy pre-existing target.
            Destroy(targetActor);
        }

        Transform[] children = sinks.GetComponentsInChildren<Transform>();
        GameObject randomGO = (GameObject)((Transform)children[Random.Range(1, children.Length)]).gameObject;

        if ("targets" != randomGO.name)
        {
            // Spawn target.
            targetActor = new GameObject(name);

            // Add relevant components.
            targetActor.AddComponent<MeshFilter>();
            targetActor.AddComponent<MeshCollider>();
            targetActor.AddComponent<Rigidbody>();
            targetActor.GetComponent<Rigidbody>().useGravity = true;
            targetActor.GetComponent<Rigidbody>().isKinematic = false;
            targetActor.GetComponent<Rigidbody>().mass = 1.0f;
            targetActor.GetComponent<Rigidbody>().angularDrag = 0.0f;
            MeshRenderer mr = targetActor.AddComponent<MeshRenderer>();
            targetActor.transform.parent = PouringAgent.transform;
            targetActor.transform.position = Workspace.transform.position + new Vector3(0, Workspace.GetComponent<Collider>().bounds.max.y, 0);
            targetActor.transform.rotation = Quaternion.Euler(Vector3.zero);
            targetActor.transform.localScale = new Vector3(15.0f, 15.0f, 15.0f);
            mr.sharedMaterial = randomGO.GetComponent<MeshRenderer>().material;
            targetActor.GetComponent<MeshFilter>().sharedMesh = randomGO.GetComponent<MeshFilter>().mesh;
            targetActor.GetComponent<MeshCollider>().sharedMesh = randomGO.GetComponent<MeshFilter>().mesh;
            targetActor.GetComponent<MeshCollider>().convex = true;
        }
    }

    /// <summary>
    /// Creates the FLex liquid array actor. Created actor is a child of source solid actor.
    /// </summary>
    private void createLiquidActor()
    {
        bool isReset = sourceActor == null;
        if (!isReset)
        {
            if (liquidActor != null)
            {
                // Destroy pre-existing liquid if any.
                Destroy(liquid);
            }
            // Get source position.
            Vector3 sourcePosition = sourceActor.GetComponent<MeshCollider>().bounds.center;
            float sourceMaxY = sourceActor.GetComponent<Collider>().bounds.max.y;
            float sourceMinY = sourceActor.GetComponent<Collider>().bounds.min.y;
            float offset = (sourceMaxY - sourceMinY) / 1.0f;
            // Create new game object for liquid and link it as a child to the source solid actor.
            liquid = new GameObject("liquid_" + EnvironmentAgent.gameObject.name);
            liquid.transform.parent = sourceActor.transform;

            // Adjust transformations for the spawned liquid.
            liquid.transform.position = sourcePosition + new Vector3(0, offset, 0);
            liquid.transform.rotation = PouringAgent.transform.rotation;

            // Add relevant components.
            liquidActor = liquid.AddComponent<FlexArrayActor>();
            liquidActor.container = flexParticleContainer;
            liquidActor.asset = liquidAsset;
            liquidActor.fluid = true;
            liquidActor.drawParticles = false;
            liquidActor.enabled = false;
            liquidActor.enabled = true;
            liquidActor.massScale = 10;

            liquid.AddComponent<FlexFluidRenderer>(); // Must for rendering fluid in the scene.

            liquidPC = liquid.AddComponent<FlexParticleController>(); // Automatically adds Particle System. Provides abstraction for controlling particle system.
            liquidPS = liquid.GetComponent<ParticleSystem>();
            liquid.GetComponent<ParticleSystemRenderer>().enabled = false; // Always keep this off. Rendering handeled by FlexFluidRenderer.
            m_particles = new ParticleSystem.Particle[liquidAsset.maxParticles]; // Initialize the particle array with length equal to maximum liquid particles.
            target_to_fill = (int) (fractionToFill * liquidAsset.maxParticles);
        }
    }

    /// <summary>
    /// Randomises spawn location for source solid acor at the begining of each episode. 
    /// </summary>
    /// <returns> Spawn position from randomly sampled x, y and z coordinates. </returns>
    private Vector3 randomiseSourceLocations()
    {
        return new Vector3(Random.Range(-2.0f, 2.0f), Random.Range(7.0f, 9.0f), Random.Range(-2.0f, 2.0f)) + EnvironmentAgent.transform.position;
    }

    /// <summary>
    /// Method to keep track of the liquid particle system. Particles are either in the source actor, or the target actor. Those not in
    /// either are considered as spilled. 
    /// </summary>
    /// <returns> Dictionary with 3 items identifying number of particles with source, target or spilled. </returns>
    private Dictionary<string, int> liquidParticleState()
    {
        bool isReset = sourceActor == null;
        if (!isReset)
        {
            // float max_fluid_level_in_target = -3.0f;
            int numParticlesAlive = liquidPS.GetParticles(m_particles);
            int numParticlesInSource = 0;
            int numParticlesInTarget = 0;
            int numParticlesBetweenSourceAndTarget = 0;
            int numParticlesSpilled = 0;

            // Get bounds for source actor. 
            Collider sourceCollider = sourceActor.GetComponent<Collider>();
            Bounds sourceBounds = sourceCollider.bounds;

            // Get bounds for target actor.
            Collider targetCollider = targetActor.GetComponent<Collider>();
            Bounds targetBounds = targetCollider.bounds;

            if (numParticlesAlive > 0)
            {
                for (int i = 0; i < numParticlesAlive; i++)
                {
                    // Flag to check if the current particle is in the source actor.
                    bool containedInSource = (m_particles[i].position.x >= sourceBounds.min.x && m_particles[i].position.x <= sourceBounds.max.x) &&
                    (m_particles[i].position.y >= sourceBounds.min.y && m_particles[i].position.y <= sourceBounds.max.y) &&
                    (m_particles[i].position.z >= sourceBounds.min.z && m_particles[i].position.z <= sourceBounds.max.z);

                    // Flag to check if the current particle is in the target actor.
                    bool containedInTarget = (m_particles[i].position.x >= targetBounds.min.x && m_particles[i].position.x <= targetBounds.max.x) &&
                    (m_particles[i].position.y >= targetBounds.min.y && m_particles[i].position.y <= targetBounds.max.y) &&
                    (m_particles[i].position.z >= targetBounds.min.z && m_particles[i].position.z <= targetBounds.max.z);

                    bool betweenSourceAndTarget = (m_particles[i].position.x >= targetBounds.min.x && m_particles[i].position.x <= sourceBounds.max.x) &&
                    (m_particles[i].position.y >= targetBounds.min.y && m_particles[i].position.y <= sourceBounds.max.y) &&
                    (m_particles[i].position.z >= targetBounds.min.z && m_particles[i].position.z <= sourceBounds.max.z);
                    
                    if (containedInSource)
                    {
                        numParticlesInSource++;
                    }
                    else if (containedInTarget)
                    {
                        numParticlesInTarget++;
                    } else if(betweenSourceAndTarget) {
                        numParticlesBetweenSourceAndTarget++;
                    }
                    else
                    {
                        // Those not contained in either source or target are spilled.
                        numParticlesSpilled++;
                    }
                }
            }

            // Update particle system with the alive particles.
            liquidPS.SetParticles(m_particles, numParticlesAlive);
            // Update results and return.
            particleDictionary["numParticlesInSource"] = numParticlesInSource;
            particleDictionary["numParticlesInTarget"] = numParticlesInTarget;
            particleDictionary["numParticlesPouring"] = numParticlesBetweenSourceAndTarget;
            particleDictionary["numParticlesSpilled"] = numParticlesSpilled;
        } else {
            particleDictionary["numParticlesInSource"] = 0;
            particleDictionary["numParticlesInTarget"] = 0;
            particleDictionary["numParticlesPouring"] = 0;
            particleDictionary["numParticlesSpilled"] = 0;
        }
        return particleDictionary;
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

        particleDictionary = new Dictionary<string, int>();

        particleDictionary.Add("numParticlesInSource", 0);
        particleDictionary.Add("numParticlesInTarget", 0);
        particleDictionary.Add("numParticlesPouring", 0);
        particleDictionary.Add("numParticlesSpilled", 0);
    }

    /// <summary>
    /// Reset the scene.
    /// </summary>

    private float SignedVolumeOfTriangle(Vector3 p1, Vector3 p2, Vector3 p3)
     {
         float v321 = p3.x * p2.y * p1.z;
         float v231 = p2.x * p3.y * p1.z;
         float v312 = p3.x * p1.y * p2.z;
         float v132 = p1.x * p3.y * p2.z;
         float v213 = p2.x * p1.y * p3.z;
         float v123 = p1.x * p2.y * p3.z;
 
         return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
     }
 
     private float VolumeOfMesh(MeshFilter meshFilter)
     {
        Mesh mesh = meshFilter.mesh;
        float volume = 0;
 
         Vector3[] vertices = mesh.vertices;
         int[] triangles = mesh.triangles;
 
         for (int i = 0; i < triangles.Length; i += 3)
         {
             Vector3 p1 = vertices[triangles[i + 0]];
             Vector3 p2 = vertices[triangles[i + 1]];
             Vector3 p3 = vertices[triangles[i + 2]];
             volume += SignedVolumeOfTriangle(p1, p2, p3);
         }
        volume *= meshFilter.gameObject.transform.localScale.x * meshFilter.gameObject.transform.localScale.y * meshFilter.gameObject.transform.localScale.z;
        return Mathf.Abs(volume);
     }
    private void ResetScene()
    {
        timer = 0.0f;
        target_filled = false;
        fractionToFill = Random.Range(0.10f, 1.00f);
        ResetLiqParticleState();
        createSolidActors(sinkName);
        createSolidActors(sourceName);
        CreateLiquidParticleContainer();
        createLiquidActor();
        collisionWithObjects = false;
        stoppingTrigger = false;
    }

    /// <summary>
    /// Called once. Initialises names, desired landing length and resets the scene.
    /// </summary>
    private void StartScene()
    {
        sourceName = "SourceObject_" + EnvironmentAgent.gameObject.name;
        sinkName = "TargetObject_" + EnvironmentAgent.gameObject.name;
    }

    /// <summary>
    /// Performs pouring action. Called after every action received to change the state.
    /// </summary>
    /// <param name="x"> Suggested x position to move to by the action.</param>
    /// <param name="z"> Suggested z position to move to by the action.</param>
    /// <param name="sourceVelocity"> Suggested speed of the source actor by the action for movement.</param>
    /// <param name="sourceRotVelocity"> Suggested rotational speed of the source actor for pouring.</param>
    private void performPouringAction(float x, float z, float distanceMultiplier, float sourceVelocity, float sourceRotVelocity)
    {
        // Check if scene is in reset stage.
        bool isReset = (sourceActor == null) || (targetActor == null);
        if (!isReset)
        {
            if (stoppingTrigger)
            {
                // Reached atmost desired radius. Begin rotational action now.
                if (!target_filled)
                {
                    Dictionary<string, int> liqPS = liquidParticleState();
                    // Pour until 30% of target not reached.
                    if (liqPS["numParticlesInTarget"] < target_to_fill)
                    {
                        // Begin pouring
                        Vector3 relativePositionOfDestination = sourceActor.transform.position - targetActor.transform.position;
                        Vector3 axisOfRotation = Vector3.Cross(Vector3.up, -relativePositionOfDestination.normalized);
                        float stepSize = Mathf.Lerp(sourceRotVelocity * Time.deltaTime, (sourceRotVelocity * Time.deltaTime) / 5.0f, Time.deltaTime);
                        // Reward pouring from source to target
                        int total = (liqPS["numParticlesInSource"] + liqPS["numParticlesInTarget"] + liqPS["numParticlesSpilled"] + liqPS["numParticlesPouring"]);
                        SetReward(total > 0 ? (float)liqPS["numParticlesPouring"] / total : 0);
                        if (liqPS["numParticlesInTarget"] >= (int)(target_to_fill * turnAroundLevel)) {
                            if(1.0f - Quaternion.Dot(sourceActor.transform.rotation, EnvironmentAgent.transform.rotation) <= 1e-4) {
                                target_filled = true;
                            } else {
                                sourceActor.transform.Rotate(axisOfRotation, -5 * sourceRotVelocity * Time.deltaTime, Space.Self);
                            }
                        } else {
                            sourceActor.transform.Rotate(axisOfRotation, stepSize, Space.Self);
                        }
                        float ratioParticlesSpilled = (float)liqPS["numParticlesSpilled"] / (liqPS["numParticlesInSource"] + liqPS["numParticlesInTarget"] + liqPS["numParticlesSpilled"] + liqPS["numParticlesPouring"]);
                        if (ratioParticlesSpilled > 0.15f) {
                            print("Poured: " + liqPS["numParticlesInTarget"] + " target: " + target_to_fill + " difference: " + (Mathf.Abs(liqPS["numParticlesInTarget"] - target_to_fill)));
                            // reward += -10f * distanceMultiplier;
                            SetReward(-10f * distanceMultiplier);
                            EndEpisode();
                        }
                    } else {
                        target_filled = true;
                    }
                }
                else
                {
                    // Complete rotation made. Add rewards and end episode.
                    timer += Time.deltaTime;
                    if (timer > 9.0f)
                    {
                        Dictionary<string, int> liqPS = liquidParticleState();
                        print("Poured: " + liqPS["numParticlesInTarget"] + " target: " + target_to_fill+ " difference: " + (Mathf.Abs(liqPS["numParticlesInTarget"] - target_to_fill)));
                        // Reward if desired level reached with some tolerance.
                        int differenceFromTarget = (Mathf.Abs(liqPS["numParticlesInTarget"] - target_to_fill));
                        // float targetReward = 0.0f;
                        // 1. Reward all actions such that the difference reduces.
                        // targetReward = 100.0f * -differenceFromTarget / liquidAsset.maxParticles;
                        SetReward(differenceFromTarget <= 25 ? 10.0f * differenceFromTarget / liquidAsset.maxParticles : 20.0f * -differenceFromTarget / liquidAsset.maxParticles);
                        // 2. Reward turn around level action if difference is tolerable. Penalize the rest.
                        // targetReward += differenceFromTarget <= 25 ? 100.0f * turnAroundLevel : -5.0f * 100.0f * turnAroundLevel;
                        SetReward(differenceFromTarget <= 25 ? 0.1f * turnAroundLevel * (float)(target_to_fill) : -0.2f * (turnAroundLevel * (float)(target_to_fill)));
                        // 3. Reward all action rotation velocities if difference is tolerable. Penalise the rest.
                        // targetReward += differenceFromTarget <= 25 ? 10.0f * sourceRotVelocity : -50.0f * sourceRotVelocity;
                        SetReward(differenceFromTarget <= 25 ? 10.0f * sourceRotVelocity : -50.0f * sourceRotVelocity);
                        // reward += targetReward;
                        // SetReward(reward);
                        EndEpisode();
                    } 
                }
            } else {
                MeshCollider targetMesh = targetActor.GetComponent<MeshCollider>();
                Bounds bounds = targetMesh.bounds;
                desiredRadius = Vector3.Distance(bounds.min, bounds.max) * distanceMultiplier;

                // 1. Reach appropriate position.
                // Check if spawned source is within the periphery of the target actor.
                bool withinTargetPerimeter = (sourceActor.transform.position.x >= bounds.min.x &&
                                            sourceActor.transform.position.x <= bounds.max.x) &&
                                            (sourceActor.transform.position.z >= bounds.min.z &&
                                            sourceActor.transform.position.z <= bounds.max.z);
                Vector3 suggestedPosition = Vector3.zero;
                suggestedPosition.x = x;
                suggestedPosition.y = sourceActor.transform.position.y;
                suggestedPosition.z = z;

                suggestedPosition += EnvironmentAgent.transform.position;

                // Track distances of source actor and suggested in XZ plane.  
                Vector2 xzPlanarCoordinatesSource = Vector2.zero;
                xzPlanarCoordinatesSource.x = sourceActor.transform.position.x;
                xzPlanarCoordinatesSource.y = sourceActor.transform.position.z;

                Vector2 xzPlanarCoordinatesTarget = Vector2.zero;
                xzPlanarCoordinatesTarget.x = targetActor.transform.position.x;
                xzPlanarCoordinatesTarget.y = targetActor.transform.position.z;
                float currentDistance = Vector2.Distance(xzPlanarCoordinatesSource, xzPlanarCoordinatesTarget);
                timer += Time.deltaTime;
                if (timer > 4.0f)
                {
                    if (withinTargetPerimeter)
                    {
                        sourceActor.transform.position = Vector3.MoveTowards(sourceActor.transform.position, suggestedPosition, -sourceVelocity * Time.deltaTime);
                        stoppingTrigger = currentDistance > desiredRadius;
                        float postActionDistance = Vector2.Distance(new Vector2(sourceActor.transform.position.x, sourceActor.transform.position.z), xzPlanarCoordinatesTarget);

                        // If Object already within target's perimeter boundary, reward actions that increase distance.
                        // reward += postActionDistance > currentDistance ? postActionDistance : -postActionDistance;
                        SetReward(postActionDistance > currentDistance ? postActionDistance : -postActionDistance);
                    }
                    else
                    {
                        sourceActor.transform.position = Vector3.MoveTowards(sourceActor.transform.position, suggestedPosition, sourceVelocity * Time.deltaTime);
                        stoppingTrigger = currentDistance < desiredRadius;
                        float postActionDistance = Vector2.Distance(new Vector2(sourceActor.transform.position.x, sourceActor.transform.position.z), xzPlanarCoordinatesTarget);

                        // If Object already outside the target's perimeter boundary, reward actions that decreases distance.
                        // reward += postActionDistance < currentDistance ? postActionDistance : -postActionDistance;
                        SetReward(postActionDistance < currentDistance ? postActionDistance : -postActionDistance);
                    }

                    // Reward if during movement ratio of particles in source is >= 90% (Some liquid particles may splash while initial rendering)
                    Dictionary<string, int> liqPS = liquidParticleState();
                    float ratioParticlesInSource = (float)liqPS["numParticlesInSource"] / (liqPS["numParticlesInSource"] + liqPS["numParticlesInTarget"] + liqPS["numParticlesSpilled"] + 
                                                    liqPS["numParticlesPouring"]);
                    if (ratioParticlesInSource < 0.90f) {
                        // Penalise all velocities which results in less than 80% of liquid in source and end episode.
                        // reward += - 10.0f * sourceVelocity;
                        SetReward(- 10.0f * sourceVelocity);
                        EndEpisode();
                    } else
                    {
                        // Reward if while moving, source retained 95% of liquid particles.
                        // reward += ratioParticlesInSource >= 95.0f ? sourceVelocity / 10.0f : -5.0f * (sourceVelocity / 10.0f);
                        SetReward(ratioParticlesInSource >= 95.0f ? sourceVelocity / 10.0f : -5.0f * (sourceVelocity / 10.0f));
                    }
                }
            }
        }
    }


    /// <summary>
    /// Called once when ML agents training start. Initializes academy parameters and starts the scene.
    /// </summary>
    public override void Initialize()
    {
        defaultAcademyParameters = Academy.Instance.EnvironmentParameters;
        StartScene();
    }

    /// <summary>
    /// Adds sensor observations used by PPO RL Agent to make actions.
    /// Overall 9 observations.
    /// </summary>
    /// <param name="sensor"> Vector sensor observations.</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        if (sourceActor != null)
        {
            sensor.AddObservation((float)target_to_fill / liquidAsset.maxParticles); // 1 observation for target to fill as a ratio of total particles
            sensor.AddObservation(turnAroundLevel); // 1 Observation for turn around level suggested by actions.
            sensor.AddObservation(particleDictionary["numParticlesSpilled"] / liquidAsset.maxParticles); // 1 observation for ratio of particles spilling
            sensor.AddObservation(particleDictionary["numParticlesPouring"] / liquidAsset.maxParticles); // 1 observation for ratio of particles pouring from source to sink
            sensor.AddObservation(particleDictionary["numParticlesInSource"] / liquidAsset.maxParticles); // 1 observation for ratio of particles in source
            sensor.AddObservation(particleDictionary["numParticlesInTarget"] / liquidAsset.maxParticles); // 1 observation for ratio of particles in target
            sensor.AddObservation(suggestedSourceRotationalVelocity / 100f); // 1 Observation for suggested rotational velocity
            sensor.AddObservation(suggestedSourceVelocity / 10f); // 1 Observation for suggested source velocity
            sensor.AddObservation(suggestedX / 10f);
            sensor.AddObservation(suggestedZ / 10f); // 2 observations for suggested moving position.
            sensor.AddObservation(collisionWithObjects); // 1 Observation for collision.
            sensor.AddObservation(flexParticleContainer.viscosity / 100f); // 1 observation for liquid viscosity.
            sensor.AddObservation(desiredRadius); // 1 observation of desired landing zone.
            float minTargetRimDim = Mathf.Min(Mathf.Abs(sourceActor.GetComponent<Collider>().bounds.max.x - sourceActor.GetComponent<Collider>().bounds.min.x),
                                    Mathf.Abs(sourceActor.GetComponent<Collider>().bounds.max.z - sourceActor.GetComponent<Collider>().bounds.min.z));
            float rimArea = Mathf.PI * minTargetRimDim * minTargetRimDim;
            sensor.AddObservation(rimArea); // 1 observation for approximate cirucalr rim areaof the source container.
        }
    }

    /// <summary>
    /// Tasks to do when episode begins.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        ResetScene();
    }

    float suggestedSourceVelocity;
    float suggestedSourceRotationalVelocity;
    float suggestedX;
    float suggestedZ;

    float suggestedLandingMultiplier;

    /// <summary>
    /// Tasks to do when an action is received from the PPO. Actions are continuous domain. 
    /// </summary>
    /// <param name="actions"> Actions received from PPO agent.</param>
    public override void OnActionReceived(ActionBuffers actions)
    {

        int actionIndex = -1;

        suggestedX = 10f * Mathf.Clamp(actions.ContinuousActions[++actionIndex], -1f, 1f); // Suggested x position for source.
        suggestedZ = 10f * Mathf.Clamp(actions.ContinuousActions[++actionIndex], -1f, 1f); // Suggested z position for source.
        suggestedLandingMultiplier = 1.5f * Mathf.Clamp(actions.ContinuousActions[++actionIndex], 0.2f, 1f); // Suggested multipler for desired landing distance
        suggestedSourceVelocity = 10f * Mathf.Clamp(actions.ContinuousActions[++actionIndex], 0.1f, 0.2f); // Suggested velocity of source vessle.
        suggestedSourceRotationalVelocity = 10f * Mathf.Clamp(actions.ContinuousActions[++actionIndex], 0.2f, 0.5f); // Suggested rotation velocity of source vessle. 
        turnAroundLevel = Mathf.Clamp(actions.ContinuousActions[++actionIndex], 0.85f, 0.95f); // Suggested turn around level
    }
    /// <summary>
    /// Tasks to perform. It's called each step once.
    /// </summary>
    private void FixedUpdate()
    {
        performPouringAction(suggestedX, suggestedZ, suggestedLandingMultiplier,  suggestedSourceVelocity, suggestedSourceRotationalVelocity);
    }

    private void OnGUI() {
        Dictionary<string, int> liqPS = liquidParticleState();
        GUI.Label(new Rect(10, 10, 1000, 20), "Poured: " + liqPS["numParticlesInTarget"] + " target: " + target_to_fill+ " difference: " + (Mathf.Abs(liqPS["numParticlesInTarget"] - target_to_fill)));   
    }
}
