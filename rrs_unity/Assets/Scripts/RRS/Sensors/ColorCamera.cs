using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetMQ;
using UnityEngine.Rendering;
using Unity.Collections;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using Simulator.Plugins;
using System;

//LMT GPU Enabled Camera Simulator

public class ColorCamera : MonoBehaviour
{
   
    [Header("Shader Setup")]
    public Shader uberReplacementShader;
    public Shader opticalFlowShader;
    public float opticalFlowSensitivity = 1.0f;

    // cached materials
    private Material opticalFlowMaterial;

    public delegate void DelegateDataChanged(byte[] buffer);
    public event DelegateDataChanged delegateCameraDataChanged;

    protected Camera SensorCamera;

    protected RenderTextureReadWrite CameraTargetTextureReadWriteType = RenderTextureReadWrite.sRGB;

    public int CubemapSize = 1024;

    ConcurrentBag<NativeArray<byte>> AvailableGpuDataArrays = new ConcurrentBag<NativeArray<byte>>();

    private ConcurrentBag<byte[]> JpegOutput = new ConcurrentBag<byte[]>();
    private Queue<Task> Tasks = new Queue<Task>();
    private RenderTexture DistortedTexture;

    [Range(1, 1920)]
    public int Width = 800;

    [Range(1, 1080)]
    public int Height = 600;

    [Range(1, 100)]
    public int Frequency = 15;

    [Range(0, 100)]
    public int JpegQuality = 75;

    [Range(1.0f, 90.0f)]
    public float FieldOfView = 60.0f;

    [Range(0.01f, 1000.0f)]
    public float MinDistance = 0.1f;

    [Range(0.01f, 2000.0f)]
    public float MaxDistance = 1000.0f;

    const int MaxJpegSize = 4 * 1024 * 1024; // 4MB
    byte[] buffer;
    private float NextCaptureTime;

    uint Sequence;

    public enum Mode
    {
        a,b,c,d,e,f
    }

    public Mode mode;

    private struct CameraCapture
    {
        public NativeArray<byte> GpuData;
        public AsyncGPUReadbackRequest Request;
        public double CaptureTime;
    }

    private Queue<CameraCapture> CaptureQueue = new Queue<CameraCapture>();

    void Start()
    {
        SensorCamera = GetComponent<Camera>();

        buffer = new byte[MaxJpegSize];
        // default fallbacks, if shaders are unspecified
        if (!uberReplacementShader)
            uberReplacementShader = Shader.Find("Hidden/UberReplacement");

        if (!opticalFlowShader)
            opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

        OnCameraChange();
        OnSceneChange();
    }

    void CheckTexture()
    {
        // if this is not first time
        if (SensorCamera.targetTexture != null)
        {
            if (Width != SensorCamera.targetTexture.width || Height != SensorCamera.targetTexture.height)
            {
                // if camera capture size has changed
                SensorCamera.targetTexture.Release();
                SensorCamera.targetTexture = null;
            }
            else if (!SensorCamera.targetTexture.IsCreated())
            {
                // if we have lost rendertexture due to Unity window resizing or otherwise
                SensorCamera.targetTexture.Release();
                SensorCamera.targetTexture = null;
            }
        }

        if (SensorCamera.targetTexture == null)
        {
            SensorCamera.targetTexture = new RenderTexture(Width, Height, 24,
                RenderTextureFormat.ARGB32, CameraTargetTextureReadWriteType)
            {
                dimension = TextureDimension.Tex2D,
                antiAliasing = 1,
                useMipMap = false,
                useDynamicScale = false,
                wrapMode = TextureWrapMode.Clamp,
                filterMode = FilterMode.Bilinear,
            };
        }

    }

    void CheckCapture()
    {
        if (Time.time >= NextCaptureTime)
        {
            //print("Camera Render");

            SensorCamera.Render();

            NativeArray<byte> gpuData;
            while (AvailableGpuDataArrays.TryTake(out gpuData) && gpuData.Length != Width * Height * 4)
            {
                gpuData.Dispose();
            }
            if (!gpuData.IsCreated)
            {
                gpuData = new NativeArray<byte>(Width * Height * 4, Allocator.Persistent);
            }

            var capture = new CameraCapture()
            {
                GpuData = gpuData,
                CaptureTime = Time.time,
            };
            capture.Request = AsyncGPUReadback.RequestIntoNativeArray(ref capture.GpuData, SensorCamera.targetTexture, 0, TextureFormat.RGBA32);
            // TODO: Replace above AsyncGPUReadback.Request with following AsyncGPUReadback.RequestIntoNativeArray when we upgrade to Unity 2020.1
            // See https://issuetracker.unity3d.com/issues/asyncgpureadback-dot-requestintonativearray-crashes-unity-when-trying-to-request-a-copy-to-the-same-nativearray-multiple-times
            // for the detaisl of the bug in Unity.
            //capture.Request = AsyncGPUReadback.RequestIntoNativeArray(ref capture.GpuData, Distorted ? DistortedTexture : SensorCamera.targetTexture, 0, TextureFormat.RGBA32);
            CaptureQueue.Enqueue(capture);

            NextCaptureTime = Time.time + (1.0f / Frequency);
        }
    }
  
    void ProcessReadbackRequests()
    {
        while (CaptureQueue.Count > 0)
        {
            var capture = CaptureQueue.Peek();
            if (capture.Request.hasError)
            {
                CaptureQueue.Dequeue();
                AvailableGpuDataArrays.Add(capture.GpuData);
                Debug.Log("Failed to read GPU texture");
            }
            else if (capture.Request.done)
            {
                CaptureQueue.Dequeue();

                    // TODO: Remove the following two lines of extra memory copy, when we can use 
                    // AsyncGPUReadback.RequestIntoNativeArray.
                    var data = capture.Request.GetData<byte>();
                    NativeArray<byte>.Copy(data, capture.GpuData, data.Length);

                    if (!JpegOutput.TryTake(out buffer))
                    {
                       buffer = new byte[MaxJpegSize];
                    }


                    Tasks.Enqueue(Task.Run(() =>
                    {
                        int size = JpegEncoder.Encode(capture.GpuData, Width, Height, 4, JpegQuality, buffer);
                        if (size > 0)
                        {
                            //print(size);

                            byte[] new_buffer = new byte[size];
                
                            Buffer.BlockCopy(buffer, 0, new_buffer, 0, size);

                            delegateCameraDataChanged?.Invoke(new_buffer);
                        }
                        else
                        {
                            Debug.Log("Compressed image is empty, length = 0");
                        }
                        JpegOutput.Add(buffer);
                        AvailableGpuDataArrays.Add(capture.GpuData);
                    }));

                    Sequence++;
                
               
            }
            else
            {
                break;
            }
        }
    }




    public void OnCameraChange()
    {
        // cache materials and setup material properties
        if (!opticalFlowMaterial || opticalFlowMaterial.shader != opticalFlowShader)
            opticalFlowMaterial = new Material(opticalFlowShader);
        opticalFlowMaterial.SetFloat("_Sensitivity", opticalFlowSensitivity);

        if (mode == Mode.a) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.ObjectId);
        if (mode == Mode.b) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.CatergoryId);
        if ( mode == Mode.c ) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.DepthCompressed, Color.white);
        if (mode == Mode.d) SetupCameraWithReplacementShader(SensorCamera, uberReplacementShader, ReplacementMode.Normals);
        if (mode == Mode.e) SetupCameraWithPostShader(SensorCamera, opticalFlowMaterial, DepthTextureMode.Depth | DepthTextureMode.MotionVectors);

    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode)
    {
        SetupCameraWithReplacementShader(cam, shader, mode, Color.black);
    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode, Color clearColor)
    {
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
        cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.allowHDR = false;
        cam.allowMSAA = false;
    }

    static private void SetupCameraWithPostShader(Camera cam, Material material, DepthTextureMode depthTextureMode = DepthTextureMode.None)
    {
        var cb = new CommandBuffer();
        cb.Blit(null, BuiltinRenderTextureType.CurrentActive, material);
        cam.AddCommandBuffer(CameraEvent.AfterEverything, cb);
        cam.depthTextureMode = depthTextureMode;
    }

    public enum ReplacementMode
    {
        ObjectId = 0,
        CatergoryId = 1,
        DepthCompressed = 2,
        DepthMultichannel = 3,
        Normals = 4
    };

    public void OnSceneChange()
    {
        var renderers = UnityEngine.Object.FindObjectsOfType<Renderer>();
        var mpb = new MaterialPropertyBlock();
        foreach (var r in renderers)
        {
            var id = r.gameObject.GetInstanceID();
            var layer = r.gameObject.layer;
            var tag = r.gameObject.tag;

            mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
            mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
            r.SetPropertyBlock(mpb);
        }
    }

    public virtual bool Save(string path, int quality, int compression)
    {
        CheckTexture();
        SensorCamera.Render();
        var readback = AsyncGPUReadback.Request(SensorCamera.targetTexture, 0, TextureFormat.RGBA32);
        readback.WaitForCompletion();

        if (readback.hasError)
        {
            Debug.Log("Failed to read GPU texture");
            return false;
        }

        Debug.Assert(readback.done);
        var data = readback.GetData<byte>();

        var bytes = new byte[16 * 1024 * 1024];
        int length;

        var ext = System.IO.Path.GetExtension(path).ToLower();

        if (ext == ".jpeg" || ext == ".jpg")
        {
            length = JpegEncoder.Encode(data, Width, Height, 4, quality, bytes);
        }
        else
        {
            return false;
        }

        if (length > 0)
        {
            try
            {
                using (var file = System.IO.File.Create(path))
                {
                    file.Write(bytes, 0, length);
                }
                return true;
            }
            catch
            {
            }
        }

        return false;
    }

    public void OnDestroy()
    {
        if (SensorCamera != null && SensorCamera.targetTexture != null)
        {
            SensorCamera.targetTexture.Release();
        }
        if (DistortedTexture != null)
        {
            DistortedTexture.Release();
        }

        while (CaptureQueue.Count > 0)
        {
            var capture = CaptureQueue.Dequeue();
            capture.GpuData.Dispose();
        }

        // Wait all tasks finished to gurantee all native arrays are in AvailableGpuDataArrays.
        Task.WaitAll(Tasks.ToArray());
        while (AvailableGpuDataArrays.TryTake(out var gpuData))
        {
            gpuData.Dispose();
        }
    }

    private void Update()
    {
        SensorCamera.fieldOfView = FieldOfView;
        SensorCamera.nearClipPlane = MinDistance;
        SensorCamera.farClipPlane = MaxDistance;

        while (Tasks.Count > 0 && Tasks.Peek().IsCompleted)
        {
            Tasks.Dequeue();
        }

        CheckTexture();
        CheckCapture();
        ProcessReadbackRequests();
    }
}
