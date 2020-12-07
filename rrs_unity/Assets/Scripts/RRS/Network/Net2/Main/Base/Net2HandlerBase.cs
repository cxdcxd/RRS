using RRS.Tools.Log;
using RRS.Tools;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        public class Net2HandlerBase : INet2Handler, IDisposable
        {
            Net2Base net2_instance;

            bool disposed = false;

            Net2Base INet2Handler.Instance => net2_instance;

            public delegate void DelegateStateChanged(Net2HandlerBase sender);
            public event DelegateStateChanged delegateStateChanged;

            public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
            public event DelegateNewLog delegateNewLog;

            public delegate void DelegateNewData(ulong sequence, byte[] buffer, uint priority, Net2HandlerBase sender);
            public event DelegateNewData delegateNewData;

            public delegate void DelegateSendChanged(Net2HandlerBase sender);
            public event DelegateSendChanged delegateSendChanged;

            public delegate void DelegateReceiveChanged(Net2HandlerBase sender);
            public event DelegateReceiveChanged delegateReceiveChanged;

            ~Net2HandlerBase()
            {
                Dispose();
            }

            private void Net2_instance_delegateSendChanged(ulong id)
            {
                delegateSendChanged?.Invoke(this);
            }

            private void Net2_instance_delegateReceiveChanged(ulong id)
            {
                delegateReceiveChanged?.Invoke(this);
            }

            private void Net2_instance_delegateStateChanged(ulong id)
            {
                delegateStateChanged?.Invoke(this);
            }

            protected void Net2_instance_delegateNewData(ulong sequence, byte[] buffer, uint priority, ulong id)
            {
                delegateNewData?.Invoke(sequence, buffer, priority, this);
            }

            private void Net2_instance_delegateNewLog(string log_message, Log.LogType log_type, string section)
            {
                delegateNewLog?.Invoke(log_message, log_type, section);
            }

            protected virtual void _ReleaseInstance()
            {
                if (net2_instance != null)
                {
                    net2_instance.delegateNewLog -= Net2_instance_delegateNewLog;
                    net2_instance.delegateStateChanged -= Net2_instance_delegateStateChanged;
                    net2_instance.delegateReceiveChanged -= Net2_instance_delegateReceiveChanged;
                    net2_instance.delegateSendChanged -= Net2_instance_delegateSendChanged;
                    
                    net2_instance.Dispose();
                    net2_instance = null;
                }
            }

            protected virtual void _AddInstance()
            {
                if (instance != null)
                {
                    net2_instance.delegateNewLog += Net2_instance_delegateNewLog;
                    net2_instance.delegateStateChanged += Net2_instance_delegateStateChanged;
                    net2_instance.delegateReceiveChanged += Net2_instance_delegateReceiveChanged;
                    net2_instance.delegateSendChanged += Net2_instance_delegateSendChanged;
                }
            }

            void INet2Handler.SetInstance(Net2Base instance)
            {
                if (net2_instance != null)
                {
                    throw new Exception("Handler allready has a channel instance!");
                }
                net2_instance = instance;

                _AddInstance();
            }

            void INet2Handler.ReleaseInstance()
            {
                _ReleaseInstance();
            }

            public ulong Id
            {
                get
                {
                    if (net2_instance != null)
                    {
                        return net2_instance.Id;
                    }
                    else
                    {
                        return 0;
                    }
                }
            }

            public string Name
            {
                get
                {
                    if (net2_instance != null)
                    {
                        return net2_instance.Name;
                    }
                    else
                    {
                        return null;
                    }
                }
            }

            public Net2State State
            {
                get
                {
                    if (net2_instance != null)
                    {
                        return net2_instance.State;
                    }
                    else
                    {
                        return Net2State.STOPPED;
                    }
                }
            }

            public uint ConnectionCount
            {
                get
                {
                    if (net2_instance != null)
                    {
                        return net2_instance.ConnectionCount;
                    }
                    else
                    {
                        return 0;
                    }
                }
            }

            public TransmitedData BytesReceive
            {
                get
                {
                    if (net2_instance != null)
                    {
                        return net2_instance.BytesReceive;
                    }
                    else
                    {
                        return new TransmitedData();
                    }
                }
            }

            public TransmitedData BytesSend
            {
                get
                {
                    if (net2_instance != null)
                    {
                        return net2_instance.BytesSend;
                    }
                    else
                    {
                        return new TransmitedData();
                    }
                }
            }

            public void Dispose()
            {
                if (disposed)
                    return;
                net2_instance.Dispose();
                _ReleaseInstance();
                disposed = true;
            }
        }
    }
}
