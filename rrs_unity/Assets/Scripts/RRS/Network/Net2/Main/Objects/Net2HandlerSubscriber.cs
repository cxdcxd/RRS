using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        public class Net2HandlerSubscriber : Net2HandlerBase
        {
            protected override void _ReleaseInstance()
            {
                Net2Subscriber instance = (this as INet2Handler).Instance as Net2Subscriber;
                if (instance != null)
                {
                    instance.delegateNewData -= Net2_instance_delegateNewData;
                }

                base._ReleaseInstance();
            }

            protected override void _AddInstance()
            {
                Net2Subscriber instance = (this as INet2Handler).Instance as Net2Subscriber;
                if (instance != null)
                {
                    instance.delegateNewData += Net2_instance_delegateNewData;
                }

                base._AddInstance();
            }
        }
    }
}
