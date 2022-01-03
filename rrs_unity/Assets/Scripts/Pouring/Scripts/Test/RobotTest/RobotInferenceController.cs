using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;
using Unity.MLAgents;
using Unity.Barracuda;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RobotInferenceController : Agent
{
    private int configureAgnet = -1;
    public NNModel model;
    
    public GameObject workspace;
    public GameObject sources;
    public GameObject targets;
    public PhysicMaterial solidObjectPhysicsMaterial;
    public PhysicMaterial robotPhysicsMaterial;
    public GameObject left_marker;
    public GameObject right_marker;
    public Transform initial_left_marker;
    public Transform initial_right_marker;
    public GameObject sensor_right_arm;
    public GameObject coupled_gripper_right;
    public GameObject sensor_left_arm;
    public GameObject coupled_gripper_left;
    public FlexContainer liquid_flex_container;
    public FlexArrayAsset liquid_asset;

    private Solid source;
    private Solid target;
    private Liquid liquid;
    private string sourceName; // Unique name to identify source liquid container.
    private string targetName; // Unique name to identify target liquid container.
    EnvironmentParameters defaultAcademyParameters; // ML-Agent's environment parameters.
    private List<float> fill_targets = new List<float>();
    Dictionary<string, float> liquidState = new Dictionary<string, float>();
    private float target_to_fill; // Target amount of liquid to fill.
    private float differenceFillLevel; // Fill level in the target container as a ratio of the desired target.
    private float originalWeight;
    private float liquidInSource = 0.0f;
    private float error = 0.0f;
    float weightOfLiquid = 0.0f;
    bool isTest = true;
    int experiment_indexer = 0;
    private static float fixedSourceStartVolume = 250f;
    private int numEpisodes = 0;
    private int numCompletedEpisodes = 0;
    private float timer = 0.0f;
    private bool isActionDone = false;

    private GameObject clickedPourer;

    /// <summary>
    /// Creates Solid Game Object. Based on string provided, chooses source or target.
    /// </summary>
    /// <param name="name"> Name of the soild actor to be generated.</param>
    private void createSolidObjects(string name)
    {
        if (name == "SourceObject_" + gameObject.name)
        {
            source = SpawnObjects.createLiquidContainer(source, this.gameObject, sources, this.clickedPourer, name, solidObjectPhysicsMaterial, randomizePositions: false);
        }
        else if (name == "TargetObject_" + gameObject.name)
        {
            target = SpawnObjects.createLiquidContainer(target, this.gameObject, targets, null, name, solidObjectPhysicsMaterial, randomizePositions: false);
        }
        else print(name + " not configured for flex solid actor..");
    }

    private void ResetLiquidStateInformation()
    {
        if (liquidState != null)
        {
            liquidState = null;
        }

        liquidState = new Dictionary<string, float>();
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
            sensor.AddObservation(source.getObjectDepth()); // 1 observation for pouring object's height in meters.
            sensor.AddObservation(liquid.getDensity()); // 1 Observation for liquid's density.
            sensor.AddObservation(liquid.getFlexParticleContainer().cohesion); // 1 observation for cohesion of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().surfaceTension); // 1 observation for surface tension of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().viscosity); // 1 observation for normalised viscosity of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().adhesion); // 1 observation for adhesion of the fluid.

            sensor.AddObservation(target_to_fill); // 1 objservation for target weight to fill.
            sensor.AddObservation(liquidInSource); // 1 observation for weight of the liquid in source container.
            sensor.AddObservation(discharge); // 1 observation for mass flow rate. 
            sensor.AddObservation(weightOfLiquid); // 1 observation for the current liquid weight in the target container.
            sensor.AddObservation(differenceFillLevel); // 1 observation for current difference from the target level.
            sensor.AddObservation(sourceTilt / 360); //  1 observation for current tilt of the source container about it's axis of rotation. {Action feedback}
            if (numCompletedEpisodes > 0)
                sensor.AddObservation((error / numCompletedEpisodes)); // 1 observation for average pouring error.
            else
                sensor.AddObservation(0.0f); // 1 observation for average pouring error.
        }
    }

    /// <summary>
    /// Tasks to do when an action is received from the PPO. Actions are continuous domain. 
    /// </summary>
    /// <param name="actions"> Actions received from PPO agent.</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Grasping at handle.
        bool isReset = (target == null) || (source == null);
        if (!isReset)
        {
            timer += Time.deltaTime;

            liquidState = liquid.getLiquidState(source.getSolidObject(), target.getSolidObject(), workspace, liquidState);
            if (timer > 5.0f)
            {
                performPouringAction(actions);
                AddReward(-1f / MaxStep);
            }
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
    float discharge = 0.0f;
    bool retract = false;
    float retractTimer = 0;
    private void performPouringAction(ActionBuffers actionBuffers)
    {   
        // Get actions from agent's action buffer. Overall 4 actions. Move in x, y, z and rotate speed while pouring.
        int actionIndex = -1;
        float turningSpeedMultiplier = 50.0f;
        sourceTurningSpeed = turningSpeedMultiplier * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // rotate source by this angle about axis of rotation defined by relative position between source and target.
        robotMovingSpeed = 2.0f * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // Move robotic hand by this speed.

        weightOfLiquid = sensor_left_arm.GetComponent<AddForceInformationMono>().getPouredMeasuredWeight();
        
        if (weightOfLiquid > 0.01f)
            differenceFillLevel = Mathf.Abs(target_to_fill - weightOfLiquid);

        float filledLevel = (float)weightOfLiquid / target_to_fill;
        float similarity = Vector3.Dot(source.getSolidObject().transform.up, target.getSolidObject().transform.up);

        liquidInSource = sensor_right_arm.GetComponent<AddForceInformationMono>().getPourerMeasuredWeight();
        if (liquidInSource > 0.01f && originalWeight == 0.0f)
        {
            originalWeight = liquidInSource;
            if (originalWeight < target_to_fill)
            {
                float targetOffset = Random.Range(0.01f, 0.025f);
                target_to_fill = originalWeight - targetOffset;
                if (target_to_fill < 0.01f)
                {
                    target_to_fill = targetOffset;
                }
            }
        }

        if (previousStepSourceLevel > 0.0f)
        {
            discharge = previousStepSourceLevel - liquidInSource;
        }

        if (previousStepSourceLevel != liquidInSource)
        {
            previousStepSourceLevel = liquidInSource;
        }

        Bounds targetBounds = target.getSolidObject().GetComponent<Renderer>().bounds;
            

        // Update This from real gripper
        Vector3 effectCenter = right_marker.transform.position;
        
        bool withinTargetRim = (source.getSolidObject().transform.position.x > targetBounds.min.x &&
                                source.getSolidObject().transform.position.x < targetBounds.max.x &&
                                source.getSolidObject().transform.position.z > targetBounds.min.z &&
                                source.getSolidObject().transform.position.z < targetBounds.max.z);
        if (!isActionDone)
        {

            // If positive angular velocity action and not pouring, pour.
            Vector3 directionOfMovement = (target.getSolidObject().transform.position - source.getSolidObject().transform.position).normalized;
            if (!withinTargetRim)
            {
                Vector3 newSourceLocation = Vector3.zero;
                if (source.getSolidObject().transform.position.y - target.getSolidObject().transform.position.y >= 2.5f)
                {
                    float deltaDistance = robotMovingSpeed > 0.0f ? (robotMovingSpeed * Time.deltaTime) : 0.0f;
                    newSourceLocation = deltaDistance * directionOfMovement;
                } else
                {
                    directionOfMovement = (new Vector3(target.getSolidObject().transform.position.x,
                                                          source.getSolidObject().transform.position.y,
                                                          target.getSolidObject().transform.position.z) -
                                           new Vector3(source.getSolidObject().transform.position.x,
                                                       source.getSolidObject().transform.position.y,
                                                       source.getSolidObject().transform.position.z)).normalized;
                    float deltaDistance = robotMovingSpeed > 0.0f ? (robotMovingSpeed * Time.deltaTime) : 0.0f;
                    newSourceLocation = deltaDistance * directionOfMovement;
                }
                right_marker.transform.position += newSourceLocation;
            }
            else
            {
                float deltaRotation = 0.0f;
                float absDischarge = Mathf.Abs(discharge);
                if (absDischarge < 0.0001f)
                {
                    deltaRotation = sourceTurningSpeed > 0 ? sourceTurningSpeed * Time.deltaTime : 0.0f;
                }

                if (absDischarge > 0.0002f)
                {   
                     deltaRotation = sourceTurningSpeed < 0 ? sourceTurningSpeed * Time.deltaTime : 0.0f;
                }

                right_marker.transform.RotateAround(effectCenter, Vector3.forward, deltaRotation);
                sourceTilt += deltaRotation;
            }
        }

        isActionDone = (weightOfLiquid <= originalWeight) && ((weightOfLiquid > 0.01f && differenceFillLevel < 0.020f) || (weightOfLiquid > 0.01f && liquidInSource < 0.001f) ||
                (weightOfLiquid >= target_to_fill));

        if (isActionDone)
        {
            if (!retract)
            {
                retractTimer += Time.deltaTime;

                float cutoffTime = liquid.getFlexParticleContainer().viscosity < 10.0f ? 5.0f : (target_to_fill < 0.1 ? 10.0f : 15.0f);
                
                if (retractTimer < cutoffTime)
                {
                    if (sourceTurningSpeed < 0)
                    {
                        right_marker.transform.RotateAround(effectCenter, Vector3.forward, sourceTurningSpeed * Time.deltaTime);
                        sourceTilt += sourceTurningSpeed * Time.deltaTime;
                    }
                }
                else
                {
                    retract = true;
                }
            }
            else
            {
                print("Episode " + numEpisodes + ": Target: " + target_to_fill + " Filled: " + weightOfLiquid + " Difference: " + differenceFillLevel);
                numCompletedEpisodes += 1;
                error += differenceFillLevel;
                float averagePouringError = ((error / numCompletedEpisodes) * 1000);
                if (weightOfLiquid > 0.01f && differenceFillLevel <= (toleranceInPouringDeviation + 0.0025f))
                {
                    AddReward(1.0f);
                    if ((error / numCompletedEpisodes) < 0.02f)
                    {
                        AddReward(1.0f);
                    }
                }
                print("Average Pouring Error: " + averagePouringError + " grams");
                print("Episode " + numEpisodes + ": Final Reward: " + GetCumulativeReward());
                var statsRecorder = Academy.Instance.StatsRecorder;
                statsRecorder.Add("Avg. Pouring Error (grams)", averagePouringError);
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
        ResetScene();
    }

    /// <summary>
    /// Resets the scene.
    /// </summary>
    private void ResetScene()
    {
        if (configureAgnet == 1)
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
            retractTimer = 0.0f;
            toleranceInPouringDeviation = 0.02f;

            // Spawn pourer
            createSolidObjects(sourceName);
            source.getSolidObject().transform.position = initial_right_marker.transform.position;

            Transform[] right_sensor_children = coupled_gripper_right.GetComponentsInChildren<Transform>();
            foreach (Transform childTransform in right_sensor_children)
            {
                if (childTransform.gameObject.name.Contains("RHandE"))
                {
                    source.setChildOf(childTransform.gameObject);
                }
            }

            // Set force sensors to the robot grippers
            initializeForceSensorState();
          
            // Spawn tray
            Bounds workspaceBounds = workspace.GetComponent<Renderer>().bounds;
            workspace.transform.position = left_marker.transform.position + new Vector3(workspaceBounds.extents.x, 0, 0);
            if (workspace.activeSelf)
            {
                workspace.SetActive(false);
            }
            workspace.SetActive(true);
            workspace.transform.parent = coupled_gripper_left.transform;

            // Spawn target container
            createSolidObjects(targetName);
            target.getSolidObject().transform.position = workspace.transform.position + new Vector3(0.0f, 6.0f * workspaceBounds.extents.y, 0.0f);
            target.setChildOf(workspace);

            // Spawn liquid
            print("RUNNING INFERENCE EXPERIMENTS ON TRAINED AGENT!!!");
            float density = 1.0f;
            Color color = new Color(1f, 1f, 0.894117647f);
            switch (liquid_flex_container.name)
            {
                case "Water":
                    color = new Color(0.0549019608f, 0.529411765f, 0.8f);
                    density = 1.0f;
                    break;
                case "Ink":
                    color = new Color(0.0274509804f, 0.0509803922f, 0.0509803922f);
                    density = 1.0081f;
                    break;
                case "Oil":
                    color = new Color(1f, 0.996078431f, 0.250980392f);
                    density = 0.917f;
                    break;
                case "Honey":
                    color = new Color(0.996078431f, 0.776470588f, 0.0823529412f);
                    density = 1.42f;
                    break;
                case "Glycerin":
                    color = new Color(0.847058824f, 0.862745098f, 0.839215686f);
                    density = 1.26f;
                    break;
                case "Ketchup":
                    color = new Color(0.925490196f, 0.176470588f, 0.00392156863f);
                    density = 1.092f;
                    break;
            }
            color.a = 0.85f;
            liquid = SpawnObjects.createLiquid(liquid, source, this.gameObject, liquid_flex_container, liquid_asset, fixedSourceStartVolume, density);
            liquid.getFlexParticleContainer().fluidMaterial.color = color;
            print("Liquid is: " + liquid.getFlexParticleContainer().name);
            ResetLiquidStateInformation();
        }
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

    private void initializeForceSensorState()
    {
        // Left side sensors
        FixedJoint fj_left_sensor = sensor_left_arm.GetComponent<FixedJoint>();
        AddForceInformationMono fInfoLeft = sensor_left_arm.GetComponent<AddForceInformationMono>();
        
        if (fInfoLeft == null)
            fInfoLeft = sensor_left_arm.AddComponent<AddForceInformationMono>();
        
        fInfoLeft.coupled_gripper = coupled_gripper_left;
        fInfoLeft.target = workspace;
        fInfoLeft.targetWeight = target_to_fill;

        // Right side sensors
        FixedJoint fj_right_sensor = sensor_right_arm.GetComponent<FixedJoint>();
        AddForceInformationMono fInfoRight = sensor_right_arm.GetComponent<AddForceInformationMono>();
        
        if (fInfoRight == null)
            fInfoRight = sensor_right_arm.AddComponent<AddForceInformationMono>();

        fInfoRight.coupled_gripper = coupled_gripper_right;
        fInfoRight.target = source.getSolidObject();
    }

    //private void FixedUpdate()
    //{
    //    if (configureAgnet != -1)
    //    {
    //        ConfigureAgent(configureAgnet);
    //        configureAgnet = -1;
    //    }
    //}

    /// <summary>
    /// Configures the agent. Given an integer config, brain will switch from heuristics 
    /// to trained agent.
    /// </summary>
    /// <param name="config">Config.
    /// If -1 : use Heuristics.
    /// If 1:  use trained brain.
    /// </param>
    public void ConfigureAgent(int config)
    {
        configureAgnet = config;
        if (configureAgnet == -1)
        {
            SetModel("Nothing", null);
            EndEpisode();
        }
        else if (configureAgnet == 1)
        {
            ResetScene();
            //if (liquid.getFlexParticleContainer().viscosity < 10.0f) 
            SetModel("NoPSModel", model);
            //else
            //    SetModel("PouringBrain", highViscosityModel);
        }
    }

    public void setTargetToPour(string target)
    {
        float t = float.Parse(target);
        this.target_to_fill = t / 1000f;
    }

    public void setPouringContainer(GameObject pouringContainer)
    {
        this.clickedPourer = pouringContainer;
    }

    public void setSelectedLiquid(FlexContainer container)
    {
        this.liquid_flex_container = container;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        float rotationAngle = 25f * Input.GetAxis("Horizontal");
        continuousActionsOut[0] = rotationAngle;

        if (Input.GetKey(KeyCode.RightControl))
        {
            continuousActionsOut[1] = 1.0f;
        }
    }
}
