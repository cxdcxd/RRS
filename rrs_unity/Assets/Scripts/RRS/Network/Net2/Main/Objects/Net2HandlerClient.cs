using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        public class Net2HandlerClient : Net2HandlerBase
        {
            protected override void _ReleaseInstance()
            {
                Net2Client instance = (this as INet2Handler).Instance as Net2Client;
                if (instance != null)
                {
                    instance.delegateNewData -= Net2_instance_delegateNewData;
                }

                base._ReleaseInstance();
            }

            protected override void _AddInstance()
            {
                Net2Client instance = (this as INet2Handler).Instance as Net2Client;
                if (instance != null)
                {
                    instance.delegateNewData += Net2_instance_delegateNewData;
                }

                base._AddInstance();
            }

            public long Send(byte[] data, uint priority = 0)
            {
                if ((this as INet2Handler).Instance != null)
                {
                    return ((this as INet2Handler).Instance as Net2Client).Send(data, priority);
                }
                return 0;
            }

            public Net2Responder Request(byte[] data, uint timeout, uint priority = 0)
            {
                if ((this as INet2Handler).Instance != null)
                {
                    long seq = ((this as INet2Handler).Instance as Net2Client).Send(data, priority);
                    if (seq != 0)
                    {
                        return new Net2Responder(this, seq, timeout);
                    }
                }
                return null;
            }
        }
    }
}
