using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public class TransmitedData : ICloneable
    {
        ulong bytes = 0;
        ulong old_bytes = 0;

        public delegate void DelegateTransmitedChanged();
        public event DelegateTransmitedChanged delegateTransmitedChanged;

        public TransmitedData()
        {
            bytes = 0;
            old_bytes = 0;
        }

        public TransmitedData(ulong bytes)
        {
            this.bytes = bytes;
        }

        public ulong Bytes
        {
            get
            {
                return bytes;
            }
            set
            {
                bytes = (value > 0) ? value : 0;

                if (old_bytes != bytes)
                {
                    old_bytes = bytes;
                    delegateTransmitedChanged?.Invoke();
                }
            }
        }

        public object Clone()
        {
            return new TransmitedData(bytes);
        }

        public override string ToString()
        {
            Net2DataRateUnit unit = Net2DataRateUnit.B;
            ulong amount = bytes;
            ulong residual = 0;

            while (amount >= 1024 && unit != Net2DataRateUnit.PB)
            {
                unit++;
                residual = amount % 1024;
                amount /= 1024;

            }

            int fraction = (int)Math.Round(((residual / 1024.0) * 100));

            return (amount.ToString() + (fraction > 0 ? "." + fraction.ToString("00") : "") + unit.ToString());
        }

    }
}
