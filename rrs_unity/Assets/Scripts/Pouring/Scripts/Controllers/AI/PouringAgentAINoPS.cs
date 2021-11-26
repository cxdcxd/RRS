using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;


public class PouringAgentAINoPS : Agent {

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
    public GameObject workspace; // Workspace for target hand.
    public GameObject right_marker; // NMPC right marker
    public GameObject left_marker; // NMPC left marker

    /// <summary>
    /// The objects below are created and used internally during the runtime.
    /// </summary>

    private Solid source; // Source container that holds the liquid.
    private Solid target; // Target container which receives the liquid.
    private Liquid liquid; // Liquid object with domain randomized properties.
    private Robot robotHandForSource; // Robotic hand to hold the source container.
    private Robot robotHandForTarget; // Robotic hand to hold the target container.

    /// <summary>
    /// Objects used during runtime for end to end state and action tracking.
    /// </summary>    
    EnvironmentParameters defaultAcademyParameters; // ML-Agent's environment parameters.
    private string sourceName; // Unique name to identify source liquid container.
    private string targetName; // Unique name to identify target liquid container.
    private float target_to_fill; // Target amount of liquid to fill.
    private float differenceFillLevel; // Fill level in the target container as a ratio of the desired target.
    private float originalWeight; // Original quantity of liquid in source liquid container.
    Dictionary<string, float> liquidState = new Dictionary<string, float>();
    private int numEpisodes = 0; // Counter to track number of episodes.
    private int numCompletedEpisodes = 0; // Counter to track number of completed episodes.
    private float timer = 0.0f;
    private bool isActionDone = false;
    private float liquidInSource = 0.0f;
    private float error = 0.0f;
    float weightOfLiquid = 0.0f;
    private static List<float> fill_targets = new List<float>();
    bool isTest = true;
    int experiment_indexer = 0;
    private static float fixedSourceStartVolume = 250f;

    /// <summary>
    /// The methods below are used to spawn environment actors in the scene
    /// </summary>

    /// <summary>
    /// Creates Solid Game Object. Based on string provided, chooses source or target.
    /// </summary>
    /// <param name="name"> Name of the soild actor to be generated.</param>
    private void createSolidObjects(string name)
    {
        if (name == "SourceObject_" + gameObject.name)
        {
            source = SpawnObjects.createLiquidContainer(source, this.gameObject, sources, name, solidObjectPhysicsMaterial, randomizePositions: false);
        }
        else if (name == "TargetObject_" + gameObject.name)
        {
            target = SpawnObjects.createLiquidContainer(target, this.gameObject, targets, name, solidObjectPhysicsMaterial, randomizePositions: false);
        }
        else print(name + " not configured for flex solid actor..");
    }

    private void ResetLiquidStateInformation() {
        if (liquidState != null) {
            liquidState = null;
        }

        liquidState = new Dictionary<string, float>();
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
    /// Called once. Initialises names, and resets the scene.
    /// </summary>
    private void StartScene()
    {
        if (isTest)
            experiment_indexer = 0;
        
        fill_targets.Add(0.05f);
        fill_targets.Add(0.10f);
        fill_targets.Add(0.15f);
        fill_targets.Add(0.20f);
        fill_targets.Add(0.25f);
        
        sourceName = "SourceObject_" + this.gameObject.name;
        targetName = "TargetObject_" + this.gameObject.name;
    }

    /// <summary>
    /// Resets the scene.
    /// </summary>
    private void ResetScene()
    {
        // Initialise goals
        differenceFillLevel = 0.0f;
        timer = 0.0f;
        isActionDone = false;
        liquidInSource = 0.0f;
        sourceTilt = 0.0f;
        weightOfLiquid = 0.0f;
        previousStepSourceLevel = 0.0f;
        discharge = 0.0f;
        retract = false;
        toleranceInPouringDeviation = Academy.Instance.EnvironmentParameters.GetWithDefault("tolerance", 0.0f);
        // 1. Spawn robotic arms
        robotHandForSource = SpawnObjects.createRoboticArms(robotHandForSource, this.gameObject, robot, "sourceHand", robotPhysicsMaterial);
        robotHandForTarget = SpawnObjects.createRoboticArms(robotHandForTarget, this.gameObject, robot, "targetHand", robotPhysicsMaterial);

        initialPosition = robotHandForSource.getRobotHand().transform.position;
        initialRotation = robotHandForSource.getRobotHand().transform.rotation;

        // 2. Spawn solid objects
        createSolidObjects(sourceName);
        createSolidObjects(targetName);

        // // 3. align robots with solid objects
        robotHandForSource.holdObject(source, workspace, true, true);
        robotHandForTarget.holdObject(target, workspace, true, true, true);

        Vector3 fingerAPosition = robotHandForTarget.getRobotHand().transform.position - robotHandForTarget.getFingerA().transform.position;
        Vector3 fingerBPosition = robotHandForTarget.getRobotHand().transform.position - robotHandForTarget.getFingerB().transform.position;

        left_marker.transform.position = 0.5f * (fingerAPosition + fingerBPosition);
        left_marker.transform.rotation = Quaternion.Euler(robotHandForTarget.getRobotHand().transform.rotation.eulerAngles.x - 90,
                                                          robotHandForTarget.getRobotHand().transform.rotation.eulerAngles.y ,
                                                          robotHandForTarget.getRobotHand().transform.rotation.eulerAngles.z 
                                                          );

        // 4. Create liquid
        if (isTest)
        {
            print("RUNNING INFERENCE EXPERIMENTS ON TRAINED AGENT!!!");
            if (experiment_indexer == 5)
            {
                print("Completed 5 experiments!");
                Application.Quit();
            } else
            {
                target_to_fill = fill_targets[experiment_indexer];
                experiment_indexer++;
                float density = 1.0f;
                
                liquid = SpawnObjects.createLiquid(liquid, robotHandForSource, this.gameObject, flexParticleContainer, liquidAsset, fixedSourceStartVolume, density);
                print("Liquid is: " + liquid.getFlexParticleContainer().name);
                Color color = new Color(1f, 1f, 0.894117647f);
                color.a = 0.75f;
                liquid.getFlexParticleContainer().fluidMaterial.color = color;
            }
        } else {
            int choice = Random.Range(0, 8);
            target_to_fill = Random.Range(0.05f, 0.25f);
            float density = Academy.Instance.EnvironmentParameters.GetWithDefault("density", 0.0f);
            liquid = SpawnObjects.createLiquid(liquid, robotHandForSource, this.gameObject, flexParticleContainer, liquidAsset, fixedSourceStartVolume, density);
            liquid.getFlexParticleContainer().fluidRest = Academy.Instance.EnvironmentParameters.GetWithDefault("fluid_rest_distance", 0.0f);
            liquid.getFlexParticleContainer().dissipation = Academy.Instance.EnvironmentParameters.GetWithDefault("dissipation", 0.0f);
            liquid.getFlexParticleContainer().cohesion = Academy.Instance.EnvironmentParameters.GetWithDefault("cohesion", 0.0f);
            liquid.getFlexParticleContainer().surfaceTension = Academy.Instance.EnvironmentParameters.GetWithDefault("surfaceTension", 0.0f);
            liquid.getFlexParticleContainer().viscosity = Academy.Instance.EnvironmentParameters.GetWithDefault("viscosity", 0.0f);
            liquid.getFlexParticleContainer().buoyancy = Academy.Instance.EnvironmentParameters.GetWithDefault("buoyancy", 0.0f);
            liquid.getFlexParticleContainer().adhesion = Academy.Instance.EnvironmentParameters.GetWithDefault("adhesion", 0.0f);
            liquid.getFlexParticleContainer().fluidMaterial.color = Color.Lerp(Color.green, Color.blue, liquid.getFlexParticleContainer().viscosity);
        }

        originalWeight = 0.0f;
        robotHandForTarget.getRobotHand().GetComponent<ForceInformation>().targetWeight = target_to_fill;

        // 5. Reset liquid state information
        ResetLiquidStateInformation();
    }
    

    /// <summary>
    /// Tasks to do when episode begins.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        numEpisodes += 1;
        print("Episode number: " + numEpisodes);
        ResetScene();
    }

    /// <summary>
    /// Adds sensor observations used by PPO RL Agent to make actions.
    /// Overall 6 observations.
    /// </summary>
    /// <param name="sensor"> Vector sensor observations.</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        if (source != null)
        {
            sensor.AddObservation(source.getObjectDiameter()); // 1 Observation for pouring object's diamater in meters.
            sensor.AddObservation(liquid.getDensity()); // 1 Observation for liquid's density.
            sensor.AddObservation(liquid.getFlexParticleContainer().dissipation); // 1 observation for dissipation factor that makes liquid particles loose velocity on self-collision.
            sensor.AddObservation(liquid.getFlexParticleContainer().cohesion); // 1 observation for cohesion of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().surfaceTension); // 1 observation for surface tension of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().viscosity); // 1 observation for normalised viscosity of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().buoyancy); // 1 observation for the buoyancy of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().adhesion); // 1 observation for adhesion of the fluid.
            
            sensor.AddObservation(target_to_fill); // 1 objservation for target weight to fill.
            sensor.AddObservation(liquidInSource); // 1 observation for weight of the liquid in source container.
            sensor.AddObservation(discharge); // 1 observation for mass flow rate. 
            sensor.AddObservation(weightOfLiquid); // 1 observation for the current liquid weight in the target container.
            sensor.AddObservation(differenceFillLevel); // 1 observation for current difference from the target level.
            sensor.AddObservation(sourceTilt / 360); //  1 observation for current tilt of the source container about it's axis of rotation. {Action feedback}
        }
    }

    /// <summary>
    /// Tasks to do when an action is received from the PPO. Actions are continuous domain. 
    /// </summary>
    /// <param name="actions"> Actions received from PPO agent.</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Grasping at handle.
        timer += Time.deltaTime;
        liquidState = liquid.getLiquidState(source.getSolidObject(), target.getSolidObject(), liquidState);
        if (timer > 5.0f)
        {
            performPouringAction(actions);
            AddReward(-1f / MaxStep);
        }
    }

    /// <summary>
    /// Method to perform pouring action, track states and add rewards.
    /// </summary>

    float toleranceInPouringDeviation = 0.0f;
    float sourceTurningSpeed;
    float robotMovingSpeed;
    float sourceTilt = 0.0f;
    float previousStepSourceLevel = 0.0f;
    Vector3 initialPosition = Vector3.zero;
    Quaternion initialRotation = Quaternion.identity;
    float discharge = 0.0f;
    bool retract = false;
    Vector3 retractAoR = Vector3.zero;
    private void performPouringAction(ActionBuffers actionBuffers) {
        bool isReset = (target == null) || (source == null);
        if (!isReset) {
            // Get actions from agent's action buffer. Overall 4 actions. Move in x, y, z and rotate speed while pouring.
            int actionIndex = -1;

            sourceTurningSpeed = 50.0f * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // rotate source by this angle about axis of rotation defined by relative position between source and target.
            robotMovingSpeed = 2.0f * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // Move robotic hand by this speed.

            weightOfLiquid = robotHandForTarget.getRobotHand().GetComponent<ForceInformation>().getMeasuredWeight();

            if (weightOfLiquid > 0.01f)
                differenceFillLevel = Mathf.Abs(target_to_fill - weightOfLiquid);

            float filledLevel = (float)weightOfLiquid / target_to_fill;
            float similarity = Vector3.Dot(source.getSolidObject().transform.up, target.getSolidObject().transform.up);

            
            liquidInSource = robotHandForSource.getRobotHand().GetComponent<ForceInformation>().getMeasuredWeight();
            if (liquidInSource > 0.01f && originalWeight == 0.0f) {
                originalWeight = liquidInSource;
                if (originalWeight < target_to_fill) {
                    float targetOffset = Random.Range(0.01f, 0.025f);
                    target_to_fill = originalWeight - targetOffset;
                    if (target_to_fill < 0.01f) {
                        target_to_fill = targetOffset;
                    }
                }
            }
            
            if (previousStepSourceLevel > 0.0f) {
                discharge = previousStepSourceLevel - liquidInSource;
            }
            
            if (previousStepSourceLevel != liquidInSource) {
                previousStepSourceLevel = liquidInSource;
            }

            Bounds targetBounds = target.getSolidObject().GetComponent<Renderer>().bounds;
            
            bool withinTargetRim = (source.getSolidObject().transform.position.x < targetBounds.max.x &&
                                    source.getSolidObject().transform.position.x > targetBounds.min.x &&
                                    source.getSolidObject().transform.position.z < targetBounds.max.z &&
                                    source.getSolidObject().transform.position.z > targetBounds.min.z);
            Vector3 relativePositionOfTarget = (target.getSolidObject().transform.position - robotHandForSource.getRobotHand().transform.position).normalized;
            Vector3 axisOfRotation = relativePositionOfTarget.x >= 0 ? Vector3.Cross(Vector3.up, relativePositionOfTarget) : Vector3.Cross(relativePositionOfTarget, Vector3.up);

            if (!isActionDone)
            {
                // If positive angular velocity action and not pouring, pour.
                if (withinTargetRim)
                {
                    float deltaRotation = 0.0f;
                    float absDischarge = Mathf.Abs(discharge);
                    if (sourceTurningSpeed < 0 && absDischarge < 0.001f) {
                        deltaRotation = sourceTurningSpeed * Time.deltaTime;
                    }
                    if (sourceTurningSpeed < 0 && absDischarge >= 0.003f) {
                        deltaRotation = sourceTurningSpeed * Time.deltaTime;
                    }

                    robotHandForSource.getRobotHand().transform.Rotate(axisOfRotation, deltaRotation, Space.Self);
                    right_marker.transform.Rotate(axisOfRotation, deltaRotation, Space.Self);
                    sourceTilt += sourceTurningSpeed * Time.deltaTime;
                } else {
                    Vector3 relativePlanarPosition = (new Vector3(target.getSolidObject().transform.position.x,
                                                                   source.getSolidObject().transform.position.y,
                                                                   target.getSolidObject().transform.position.z) -
                                                        new Vector3(source.getSolidObject().transform.position.x,
                                                                    source.getSolidObject().transform.position.y,
                                                                    source.getSolidObject().transform.position.z)).normalized;
                    float deltaDistance = robotMovingSpeed > 0.0f ? (robotMovingSpeed * Time.deltaTime) : 0.0f;
                    Vector3 newLocation = relativePlanarPosition * deltaDistance;
                    robotHandForSource.getRobotHand().transform.position += newLocation;
                    right_marker.transform.position = robotHandForSource.getRobotHand().transform.position;
                }
            }

            if (weightOfLiquid > 0.01f && differenceFillLevel < 0.020f) {
                isActionDone = true;
            }

            if (weightOfLiquid > 0.01f && liquidInSource < 0.001f) {
                isActionDone = true;
            }
            
            if (weightOfLiquid >= target_to_fill) {
                isActionDone = true;
            } 

            if (isActionDone)
            {
                relativePositionOfTarget = (initialPosition - robotHandForSource.getRobotHand().transform.position).normalized;
                axisOfRotation = relativePositionOfTarget.x < 0.0f? Vector3.Cross(Vector3.up, relativePositionOfTarget) : Vector3.Cross(relativePositionOfTarget, Vector3.up);
                if (!retract) {
                        if (sourceTurningSpeed < 0) {
                            robotHandForSource.getRobotHand().transform.Rotate(axisOfRotation, sourceTurningSpeed * Time.deltaTime, Space.Self);
                            right_marker.transform.Rotate(axisOfRotation, sourceTurningSpeed * Time.deltaTime, Space.Self);
                            sourceTilt += sourceTurningSpeed * Time.deltaTime;
                        }
                    float angle = Quaternion.Angle(robotHandForSource.getRobotHand().transform.rotation, initialRotation);
                    if (angle < 25f) {
                        retract = true;
                    }
                }
                else
                {
                    print("Episode " + numEpisodes + ": Target: " + target_to_fill + " Filled: " + weightOfLiquid + " Difference: " + differenceFillLevel);

                    numCompletedEpisodes += 1;

                    error += differenceFillLevel;
                    float averagePouringError = ((error / numCompletedEpisodes) * 1000);

                    if (weightOfLiquid > 0.01f && differenceFillLevel <= toleranceInPouringDeviation + 0.0025f)
                    {
                        AddReward(1.0f);
                    }

                    print("Average Pouring Error: " + averagePouringError + " grams");
                    print("Episode " + numEpisodes + ": Final Reward: " + GetCumulativeReward());
                    EndEpisode();
                    var statsRecorder = Academy.Instance.StatsRecorder;
                    statsRecorder.Add("Avg. Pouring Error (grams)", averagePouringError);
                }
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        
        float rotationAngle = 25f * Input.GetAxis("Horizontal");
        continuousActionsOut[0] = rotationAngle;
        
        if (Input.GetKey(KeyCode.None)) {
            continuousActionsOut[0] = 0.0f;
        }
    }
}