using System;
using System.Numerics;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;

namespace IIR_Butterworth_C_Sharp
{
    interface IIR_Butterworth_Interface
    {

        //get analog, pre - warped frequencies
        void Freq_pre_wrapped(int type_filt, double Wnf_1, double Wnf_2);

        //convert to low-pass prototype estimate
        void Wn_f1_Wn_f2(int type_filt, double u_f1, double u_f2);

        //Get N - th order Butterworth analog lowpass prototype. It returns the poles of the analog form located on the unit circle in the left-half plane so to have a stable causal system
        void Buttap(int order_filt);

        //Calculate the coefficients of the polynomial (based on Matlab code)
        Complex[] Poly(Complex[] temp_array_poly, int col_poly);

        //Calculate the coefficients of the characteristic polynomial (Bernard Brooks' paper (2016))
        Complex[] Char_poly(Matrix<Complex> temp_matr_poly, int row_col);

        //Calculate the factorial of the given number
        int Factorial(int i);

        //Method to calculate all the possible combinations
        int[][] Combination_method(int K, int N, int comb_n);

        //Transform to state-space
        void Zp2ss(int order_filt);

        //Bilinear transformation to find discrete equivalent
        void Bilinear(Matrix<Complex> a_arma_f, Matrix<Complex> b_arma_f, Matrix<Complex> c_arma_f, Matrix<Complex> d_arma_f, double fs_f, int type_filt_f);

        //Transform to zero - pole - gain and polynomial forms
        void Zero_pole_gain(Matrix<Complex> a_arma_f, int type_filt_f, int order_filt_f, double Wn_f_f, double Bw_f);



        //Estimate the coeffients of a band-pass filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        double[][] Lp2bp(double W_f1, double W_f2, int order_filt);

        //Estimate the coeffients of a band-stop filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        double[][] Lp2bs(double W_f1, double W_f2, int order_filt);

        //Estimate the coeffients of a high-pass filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        double[][] Lp2hp(double W_f1, int order_filt);

        //Estimate the coeffients of a low-pass filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        double[][] Lp2lp(double W_f2, int order_filt);

        //Check the stability of the filter. It returns true if the filter is atable, false if it is unstable
        bool Check_stability_iir(double[][] coeff_filt);

        //Filter the data by using the Direct-Form II Transpose, as explained in the Matlab documentation
        double[] Filter_Data(double[][] coeff_filt, double[] pre_filt_signal);

    }
}
