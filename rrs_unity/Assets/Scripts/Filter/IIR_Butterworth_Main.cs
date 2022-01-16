using System;
using System.IO;
using System.Numerics;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using System.Net.Http.Headers;
using System.Linq;
using MathNet.Numerics;
using Combinatorics;
using Combinatorics.Collections;
using System.Runtime.InteropServices.ComTypes;

namespace IIR_Butterworth_C_Sharp
{
    class IIR_Butterworth_Main
    {
               

        static void Main(string[] args)
        {

            IIR_Butterworth_Interface IBI = new IIR_Butterworth_C_Sharp.IIR_Butterworth_Implementation();

            double f1 = 30;  //High Pass
            double f2 = 40; //Low Pass
            double sf = 2048;    //Sampling frequency
            int order_filt = 3; //Order
            double Nyquist_F = sf / 2;

            double[][] coeff_final = new double[2][];
            bool stability_check;

            int type_filt = 3;
            
            int coeff_numb = 0;

            //Directory where to save the pre-filt and post-filt data
            string dir = @"C:\Users\press\OneDrive\Desktop\IIR_Butterworth_C_Sharp";
            try
            {
                //Set the current directory.
                Directory.SetCurrentDirectory(dir);

            }

            catch (DirectoryNotFoundException e)
            {

                Console.WriteLine("The specified directory does not exist. {0}", e);
            
            }

            //Build the signal to be used to test the filter
            int length_test_signal = 2047;
            double[] test_signal = new double[length_test_signal];
            double[] time = new double[length_test_signal];
            double freq_data_I = 10;
            double freq_data_II = 80;

            double[] output_filt_signal = new double[length_test_signal];
            
            for (int kk = 0; kk < length_test_signal; kk++)
            {

                time[kk] = (double)kk / sf;
                test_signal[kk] = Math.Sin(2 * Math.PI * time[kk] * freq_data_I) + Math.Sin(2 * Math.PI * time[kk] * freq_data_II);

            }

            using (StreamWriter sw = new StreamWriter("Pre_Filtered_data.txt"))
            {

                for (int hh = 0; hh < length_test_signal; hh++)
                {

                    sw.WriteLine(Convert.ToString(test_signal[hh]));

                }

            }

            using (StreamWriter sw = new StreamWriter("Time_domain.txt"))
            {

                for (int hh = 0; hh < length_test_signal; hh++)
                {

                    sw.WriteLine(Convert.ToString(time[hh]));

                }

            }

            switch (type_filt)
            {
                case 0:

                    coeff_numb = 2 * order_filt + 1;


                    for (int i = 0; i < 2; i++)
                    {

                        coeff_final[i] = new double[coeff_numb];

                    }


                    coeff_final = IBI.Lp2bp(f1 / Nyquist_F, f2 / Nyquist_F, order_filt);

                    stability_check = IBI.Check_stability_iir(coeff_final);

                    output_filt_signal = IBI.Filter_Data(coeff_final, test_signal);

                    using (StreamWriter sw = new StreamWriter("Filtered_data.txt"))
                    {

                        for (int hh = 0; hh < length_test_signal; hh++)
                        {

                            sw.WriteLine(Convert.ToString(output_filt_signal[hh]));

                        }

                    }

                    if (stability_check)
                    {


                        Console.Write("The filter is stable");
                        Console.Write("\n");

                    }

                    else
                    {

                        Console.Write("The filter is unstable");
                        Console.Write("\n");

                    }

                    for (int kk = 0; kk < 2; kk++)
                    {
                        if (kk == 0)
                        {

                            Console.Write ("Numerator: ");

                        }

                        else
                        {

                            Console.Write("Denumerator: ");

                        }

                        for (int ll = 0; ll < coeff_numb; ll++)

                        {
                            Console.Write(coeff_final[kk][ll]);
                            Console.Write("\t");

                        }

                        Console.Write("\n");

                    }

                    break;

                case 1:

                    coeff_numb = 2 * order_filt + 1;


                    for (int i = 0; i < 2; i++)
                    {

                        coeff_final[i] = new double[coeff_numb];

                    }


                    coeff_final = IBI.Lp2bs(f1 / Nyquist_F, f2 / Nyquist_F, order_filt);

                    stability_check = IBI.Check_stability_iir(coeff_final);

                    output_filt_signal = IBI.Filter_Data(coeff_final, test_signal);

                    using (StreamWriter sw = new StreamWriter("Filtered_data.txt"))
                    {

                        for (int hh = 0; hh < length_test_signal; hh++)
                        {

                            sw.WriteLine(Convert.ToString(output_filt_signal[hh]));

                        }

                    }

                    if (stability_check)
                    {

                        Console.Write("The filter is stable");
                        Console.Write("\n");

                    }

                    else
                    {

                        Console.Write("The filter is unstable");
                        Console.Write("\n");

                    }

                    for (int kk = 0; kk < 2; kk++)
                    {
                        if (kk == 0)
                        {

                            Console.Write("Numerator: ");

                        }

                        else
                        {

                            Console.Write("Denumerator: ");

                        }

                        for (int ll = 0; ll < coeff_numb; ll++)

                        {
                            Console.Write(coeff_final[kk][ll]);
                            Console.Write("\t");

                        }

                        Console.Write("\n");

                    }

                    break;

                case 2:

                    coeff_numb = order_filt + 1;

                    for (int i = 0; i < 2; i++)
                    {

                        coeff_final[i] = new double[coeff_numb];

                    }


                    coeff_final = IBI.Lp2hp(f1 / Nyquist_F, order_filt);

                    stability_check = IBI.Check_stability_iir(coeff_final);

                    output_filt_signal = IBI.Filter_Data(coeff_final, test_signal);

                    using (StreamWriter sw = new StreamWriter("Filtered_data.txt"))
                    {

                        for (int hh = 0; hh < length_test_signal; hh++)
                        {

                            sw.WriteLine(Convert.ToString(output_filt_signal[hh]));

                        }

                    }

                    if (stability_check)
                    {

                        Console.Write("The filter is stable");
                        Console.Write("\n");

                    }

                    else
                    {

                        Console.Write("The filter is unstable");
                        Console.Write("\n");

                    }

                    for (int kk = 0; kk < 2; kk++)
                    {
                        if (kk == 0)
                        {

                            Console.Write("Numerator: ");

                        }

                        else
                        {

                            Console.Write("Denumerator: ");

                        }

                        for (int ll = 0; ll < coeff_numb; ll++)

                        {
                            Console.Write(coeff_final[kk][ll]);
                            Console.Write("\t");

                        }

                        Console.Write("\n");

                    }

                    break;

                case 3:

                    coeff_numb = order_filt + 1;

                    for (int i = 0; i < 2; i++)
                    {

                        coeff_final[i] = new double[coeff_numb];

                    }


                    coeff_final = IBI.Lp2lp(f2 / Nyquist_F, order_filt);

                    stability_check = IBI.Check_stability_iir(coeff_final);

                    output_filt_signal = IBI.Filter_Data(coeff_final, test_signal);

                    using (StreamWriter sw = new StreamWriter("Filtered_data.txt"))
                    {

                        for (int hh = 0; hh < length_test_signal; hh++)
                        {

                            sw.WriteLine(Convert.ToString(output_filt_signal[hh]));

                        }

                    }

                    if (stability_check)
                    {

                        Console.Write("The filter is stable");
                        Console.Write("\n");

                    }

                    else
                    {

                        Console.Write("The filter is unstable");
                        Console.Write("\n");

                    }

                    for (int kk = 0; kk < 2; kk++)
                    {
                        if (kk == 0)
                        {

                            Console.Write("Numerator: ");

                        }

                        else
                        {

                            Console.Write("Denumerator: ");

                        }

                        for (int ll = 0; ll < coeff_numb; ll++)

                        {
                            Console.Write(coeff_final[kk][ll]);
                            Console.Write("\t");

                        }

                        Console.Write("\n");

                    }

                    break;

            }


        }
    }
}
