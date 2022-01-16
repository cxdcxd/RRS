using System;
using System.Numerics;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using System.Net.Http.Headers;
using System.Linq;
using MathNet.Numerics;
using Combinatorics;
using Combinatorics.Collections;

namespace IIR_Butterworth_C_Sharp
{

    class IIR_Butterworth_Implementation : IIR_Butterworth_Interface
    {
        //Global variables
        double fs = 2;
        double u_f1;
        double u_f2;
        double Wn;
        double Bw;

        Complex[] ptemp;
        Complex complex_real = new Complex(1.0, 0.0);
        Complex complex_imag = new Complex(0.0, 1.0);
        Complex[] p;


        int temp_dim_arr_matr;
        Matrix<Complex> a;
        Complex[] b;
        Complex[] c;

        double d;

        Matrix<Complex> a_arma;
        Matrix<Complex> b_arma;
        Matrix<Complex> c_arma;
        Matrix<Complex> d_arma;

        Matrix<Complex> t1_arma;
        Matrix<Complex> t2_arma;
        Matrix<Complex> ad_arma;
        Matrix<Complex> bd_arma;
        Matrix<Complex> cd_arma;
        Matrix<Complex> dd_arma;

        double[] num_filt;   //Vector where to temporarily save the numerator
        double[] den_filt;   // Vector where to temporarily save the denumerator
        double[][] save_filt_coeff;  //Matrix where to save the numerator and denominator. First row is the numerator; second row is the denominator


        //Step 1: get analog, pre - warped frequencies
        public void Freq_pre_wrapped(int type_filt, double Wnf_1, double Wnf_2)
        {

            Bw = 0;

            switch (type_filt)
            {

                //Band-pass
                case 0:

                    u_f1 = 2 * fs * Math.Tan(Math.PI * Wnf_1 / fs);
                    u_f2 = 2 * fs * Math.Tan(Math.PI * Wnf_2 / fs);

                    break;

                //Band-stop
                case 1:

                    u_f1 = 2 * fs * Math.Tan(Math.PI * Wnf_1 / fs);
                    u_f2 = 2 * fs * Math.Tan(Math.PI * Wnf_2 / fs);


                    break;

                //Low-pass
                case 2:

                    u_f2 = 2 * fs * Math.Tan(Math.PI * Wnf_2 / fs);

                    break;

                //High-pass
                case 3:

                    u_f1 = 2 * fs * Math.Tan(Math.PI * Wnf_1 / fs);

                    break;

            }
        }


        //Step 2: convert to low-pass prototype estimate
        public void Wn_f1_Wn_f2(int type_filt, double u_f1, double u_f2)
        {

            switch (type_filt)
            {

                //Band-pass
                case 0:
                    Bw = u_f2 - u_f1;
                    Wn = Math.Sqrt(u_f1 * u_f2);

                    break;

                //Band-stop
                case 1:

                    Bw = u_f2 - u_f1;
                    Wn = Math.Sqrt(u_f1 * u_f2);

                    break;

                //Low-pass
                case 2:

                    Wn = u_f2;

                    break;

                //High-pass
                case 3:

                    Wn = u_f1;

                    break;

            }
        }


        //Step 3: Get N - th order Butterworth analog lowpass prototype
        public void Buttap(int order_filt)
        {
            double order_filt_exp = (double)order_filt;
            int temp_length_vec = 0;
            int kkk = 1;
            do
            {

                temp_length_vec++;
                kkk += 2;

            } while (kkk <= order_filt - 1);

            //Initialize the vector "ptemp" 
            ptemp = new Complex[temp_length_vec];

            int track_cell = 0;
            for (double kk = 0; kk < (double)order_filt - 1; kk += 2)
            {

                ptemp[track_cell] = Complex.Exp(complex_imag * ((Math.PI * (kk + 1)) / (2 * order_filt_exp) + Math.PI / 2));

                track_cell++;

            }

            //Initialize the vector "p"
            p = new Complex[order_filt];

            Math.DivRem(order_filt, 2, out int temp_rem);

            if (temp_rem != 0)
            {

                for (int kk = 0; kk < order_filt; kk++)
                {

                    if (kk < order_filt - 1)
                    {

                        p[kk] = complex_imag;

                    }

                    else
                    {

                        p[kk] = -complex_real;

                    }


                }

            }

            else
            {

                for (int kk = 0; kk < order_filt; kk++)
                {

                    p[kk] = complex_imag;

                }


            }

            if (order_filt > 1)
            {
                track_cell = 0;
                for (int kk = 0; kk < temp_length_vec * 2; kk += 2)
                {

                    p[kk] = ptemp[track_cell];
                    p[kk + 1] = Complex.Conjugate(ptemp[track_cell]);

                    track_cell++;

                }
            }

        }

         //Step 4: Transform to state-space
        //Intermidiate step: calculate the coefficients of the polynomial (based on Matlab code)
        public Complex[] Poly(Complex[] temp_array_poly, int col_poly)
        {

            Complex[] coeff_pol_f = new Complex[col_poly + 1];

            coeff_pol_f[0] = 1;

            for (int ll = 0; ll < col_poly; ll++)
            {

                int yy = 0;

                do
                {

                    coeff_pol_f[ll + 1 - yy] = coeff_pol_f[ll + 1 - yy] - temp_array_poly[ll] * coeff_pol_f[ll - yy];
                    yy++;

                } while (yy <= ll);

            }

            return coeff_pol_f;

        }


        //Calculate the factorial of the given number
        public int factorial(int i)
        {

            if (i <= 1)
            {

                return 1;

            }

            return i * factorial(i - 1);

        }

        //Calculate the coefficients of the characteristic polynomial (Bernard Brooks' paper (2016))
        public Complex[] Char_poly(Matrix<Complex> temp_matr_poly, int row_col)
        {

            Complex[] coeff_pol_ff = new Complex[row_col + 1];
            Matrix<Complex> temp_val = Matrix<Complex>.Build.Dense(1, 1);
            int num_det = 0;
            Matrix<Complex> temp_matr = Matrix<Complex>.Build.Dense(row_col, row_col);

            for (int kk = 0; kk < row_col + 1; kk++)

            {

                if (kk == 0)
                {

                    coeff_pol_ff[row_col - kk] = Math.Pow(-1, row_col) * temp_matr_poly.Determinant();

                }

                else
                {

                    int[][] matrix_comb;

                    temp_val[0, 0] = 0;
                    try
                    {

                        num_det = factorial(row_col) / (factorial(row_col - kk) * factorial(kk));  //Calculate the number of combinations   

                    }

                    //If the DivideByZero exception is thrown, the filter is unstable. The method returns a default value of 10^10 for the coefficients.
                    catch (System.DivideByZeroException)
                    {

                        for (int ll = 0; ll < row_col + 1; ll++)
                        {

                            coeff_pol_ff[ll] = Math.Pow(10, 10);

                        }

                        break;

                    }

                    try
                    {

                        matrix_comb = new int[num_det][];

                    }

                    //If the numerical overflow exception is thrown, the filter is unstable. The method returns a default value of 10^10 for the coefficients.
                    catch (System.OverflowException)
                    {

                        for (int ll = 0; ll < row_col + 1; ll++)
                        {

                            coeff_pol_ff[ll] = Math.Pow(10, 10);

                        }

                        break;

                    }

                    for (int ll = 0; ll < num_det; ll++)
                    {

                        matrix_comb[ll] = new int[num_det];

                    }


                    // Generate the combinations 
                    matrix_comb = Combination_method(row_col, kk, num_det);

                    for (int mm = 0; mm < num_det; mm++)

                    {

                        temp_matr = temp_matr_poly.Clone();

                        for (int pp = 0; pp < row_col; pp++)
                        {

                            temp_matr[matrix_comb[mm][0], pp] = 0;
                            temp_matr[pp, matrix_comb[mm][0]] = 0;
                            temp_matr[matrix_comb[mm][0], matrix_comb[mm][0]] = -1;

                        }

                        for (int nn = 1; nn < kk; nn++)
                        {

                            for (int pp = 0; pp < row_col; pp++)
                            {

                                temp_matr[matrix_comb[mm][nn], pp] = 0;
                                temp_matr[pp, matrix_comb[mm][nn]] = 0;
                                temp_matr[matrix_comb[mm][nn], matrix_comb[mm][nn]] = -1;

                            }

                        }

                        temp_val[0, 0] += temp_matr.Determinant();


                    }

                    coeff_pol_ff[row_col - kk] = Math.Pow(-1, row_col) * temp_val[0, 0];


                }

            }

            return coeff_pol_ff;


        }

        //Calculate the factorial of the given number
        public int Factorial(int i)
        {

            if (i <= 1)
            {

                return 1;

            }

            return i * factorial(i - 1);

        }

        //Method to calculate all the possible combinations
        public int[][] Combination_method(int N, int K, int comb_n)
        {

            var integers = new List<int> { };

            for (int kk = 0; kk < N; kk++)
            {

                integers.Add(kk);

            }

            var matrix_comb_f_temp = new Combinations<int>(integers, K).ToArray();  //Create the combinations

            int[][] matrix_comb_f = new int[comb_n][];

            for (int hh = 0; hh < comb_n; hh++)
            {

                matrix_comb_f[hh] = new int[K];

            }

            //Save the combinations in a 2D array
            for (int hh = 0; hh < matrix_comb_f_temp.Length; hh++)
            {

                for (int kk = 0; kk < K; kk++)
                {

                    matrix_comb_f[hh][kk] = matrix_comb_f_temp[hh][kk];

                }

            }

            return matrix_comb_f;

        }


        //Intermidiate step: calculate the coefficients of the polynomial (Bernard Brooks' paper (2016))
        public Complex[] Char_poly(Complex temp_matr_poly, int row_col)
        {

            Complex[] coeff_pol_ff = new Complex[row_col + 1];
            Complex temp_val = new Complex(0, 0);


            return coeff_pol_ff;

        }


        //Step 4: Transform to state-space
        public void Zp2ss(int order_filt)
        {

            //Order the pairs of complex conjugate. The pairs with the smallest real part come first. Within pairs, the ones with the negative imaginary part comes first    
            Complex temp_max;

            //Using the selection sort algorithm to order based on the real part
            double order_filt_d = (double)order_filt;
            double temp_rem;
            Math.DivRem(order_filt, 2, out int res_rem);
            if (res_rem == 0)
            {

                temp_rem = order_filt_d;

            }

            else
            {

                temp_rem = order_filt_d - 1;

            }

            int min_real;
            for (int kk = 0; kk < temp_rem - 1; kk++)
            {
                min_real = kk;

                for (int jj = kk + 1; jj < temp_rem; jj++)
                {
                    if (p[jj].Real < p[min_real].Real)
                    {

                        min_real = jj;

                        temp_max = p[kk];
                        p[kk] = p[min_real];
                        p[min_real] = temp_max;

                    }
                }
            }

            //Using the selection sort algorithm to order the values based on the imaginary part
            for (int kk = 0; kk < temp_rem - 1; kk += 2)
            {
                min_real = kk;


                if (p[kk].Imaginary > p[kk + 1].Imaginary)
                {

                    min_real = kk + 1;

                    temp_max = p[kk];
                    p[kk] = p[min_real];
                    p[min_real] = temp_max;

                }
            }


            // Initialize state - space matrices for running series
            d = 1;

            int track_index = 1;

            // Take care of any left over unmatched pole pairs.
            // H(s) = 1 / (s ^ 2 + den(2)s + den(3))
            Complex[] temp_poly_p;


            if (order_filt > 1)
            {

                double[] b1 = new double[] { 1, 0 };
                double[] c1 = new double[] { 0, 1 };

                Math.DivRem(order_filt, 2, out int rem_div);
                temp_rem = (double)rem_div;
                int order_filt_temp;
                int dim_matr;
                Matrix<Complex> temp_matrix_a;    //Temporary matrix where to save the coefficients at each interaction
                int coeff_numb = 3;

                if (temp_rem == 0)
                {

                    order_filt_temp = order_filt;
                    dim_matr = order_filt_temp / 2;

                    temp_matrix_a = Matrix<Complex>.Build.Dense(dim_matr, coeff_numb);


                }

                else
                {

                    order_filt_temp = order_filt - 1;
                    dim_matr = order_filt_temp / 2;

                    temp_matrix_a = Matrix<Complex>.Build.Dense(dim_matr, coeff_numb);

                }

                int track_cycles = 0;
                int temp_val_pos_check;
                int dim_poly_p = 2;


                temp_poly_p = new Complex[dim_poly_p];

                temp_dim_arr_matr = dim_poly_p + (order_filt % 2);

                while (track_index < order_filt_temp)
                {

                    for (int rr = track_index - 1; rr < track_index + 1; rr++)
                    {

                        temp_poly_p[rr - track_index + 1] = p[rr];

                    }

                    Complex[] coeff_pol = new Complex[dim_poly_p + 1];

                    coeff_pol = Poly(temp_poly_p, dim_poly_p);


                    for (int qq = 0; qq < coeff_numb; qq++)
                    {

                        temp_matrix_a[track_cycles, qq] = -(coeff_pol[qq].Real);

                    }

                    //Update the state-space arrays/matrix
                    track_cycles += 1;

                    a = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, temp_dim_arr_matr);

                    int track_index_coeff = 0;

                    Math.DivRem(order_filt, 2, out int div_res);
                    if (div_res == 0)
                    {
                        ///////////////////////////////////////////////////////////////////////////////////////////////
                        //Even number of poles
                        for (int kk = 0; kk < temp_dim_arr_matr; kk++)
                        {

                            for (int gg = 0; gg < temp_dim_arr_matr; gg++)

                            {

                                temp_val_pos_check = kk - gg;

                                switch (temp_val_pos_check)
                                {
                                    case 1:

                                        a[kk, gg] = 1;

                                        break;


                                    case 0:

                                        Math.DivRem(kk + 1, 2, out int div_rem_I);
                                        if ((div_rem_I != 0) | (kk == 0))
                                        {

                                            a[kk, gg] = temp_matrix_a[track_index_coeff, 1];

                                        }

                                        else
                                        {

                                            a[kk, gg] = 0;

                                        }

                                        break;

                                    case -1:

                                        Math.DivRem(kk + 1, 2, out int div_rem_II);
                                        if ((div_rem_II != 0) | (kk == 0))
                                        {

                                            a[kk, gg] = temp_matrix_a[track_index_coeff, 2];
                                            track_index_coeff++;

                                        }

                                        else
                                        {

                                            a[kk, gg] = 0;

                                        }
                                        break;

                                    default:

                                        a[kk, gg] = 0;

                                        break;


                                }

                            }

                        }
                        ///////////////////////////////////////////////////////////////////////////////////////////////
                    }

                    else
                    {
                        ///////////////////////////////////////////////////////////////////////////////////////////////
                        //Odd number of poles
                        for (int kk = 0; kk < temp_dim_arr_matr; kk++)
                        {

                            for (int gg = 0; gg < temp_dim_arr_matr; gg++)

                            {

                                temp_val_pos_check = kk - gg;

                                switch (temp_val_pos_check)
                                {
                                    case 1:

                                        a[kk, gg] = 1;

                                        break;


                                    case 0:

                                        if (kk == 0)
                                        {

                                            a[kk, gg] = -1;

                                        }

                                        else
                                        {

                                            Math.DivRem(kk + 1, 2, out int div_rem_III);
                                            if (div_rem_III == 0)
                                            {

                                                a[kk, gg] = temp_matrix_a[track_index_coeff, 1];

                                            }

                                            else
                                            {

                                                a[kk, gg] = 0;

                                            }
                                        }

                                        break;

                                    case -1:

                                        Math.DivRem(kk + 1, 2, out int div_rem_IV);
                                        if (div_rem_IV == 0)
                                        {

                                            a[kk, gg] = temp_matrix_a[track_index_coeff, 2];
                                            track_index_coeff++;

                                        }

                                        else
                                        {

                                            a[kk, gg] = 0;

                                        }
                                        break;

                                    default:

                                        a[kk, gg] = 0;

                                        break;


                                }

                            }

                        }

                    }
                    ///////////////////////////////////////////////////////////////////////////////////////////////

                    //Initialize the vectors "b" and "c"
                    b = new Complex[temp_dim_arr_matr];
                    c = new Complex[temp_dim_arr_matr];

                    for (int kk = 0; kk < temp_dim_arr_matr; kk++)
                    {

                        if (kk == 0)
                        {

                            b[kk] = 1;

                        }

                        else
                        {

                            b[kk] = 0;

                        }

                    }

                    for (int kk = 0; kk < temp_dim_arr_matr; kk++)
                    {

                        if (kk == temp_dim_arr_matr - 1)
                        {

                            c[kk] = 1;

                        }

                        else
                        {

                            c[kk] = 0;

                        }

                    }

                    track_index += 2;

                    if (track_index < order_filt_temp)
                    {

                        //Clean up the matrix "a" and the arrays "b" and "c", so they can be re-initialized
                        a.Clear();
                        Array.Clear(b, 0, b.Length);
                        Array.Clear(b, 0, c.Length);

                    }

                    dim_matr += 2;
                    temp_dim_arr_matr += 2;

                }

            }

            else
            {

                a = Matrix<Complex>.Build.Dense(1, 1);
                a[0, 0] = p[0];

                b = new Complex[1];
                b[0] = 1;
                c = new Complex[1];
                c[0] = 1;

            }

            d = 0;


        }

        //Estimate the coeffients of a band-pass filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        public double[][] Lp2bp(double W_f1, double W_f2, int order_filt)
        {
            
            //Check that the low cut-off frequency is higher than the low cut-off frequency
            if (W_f2 <= W_f1)
            {

                throw new Exception("The low cut-off frequency needs to be higher than the low cut-off frequency");

            }
            
            //Check that the normalized frequencies are within the correct range of values
            if (W_f1 <= 0 | W_f1 >= 1 | W_f2 <= 0 | W_f2 >= 1)
            {

                throw new Exception("Cut-off frequencies must be in the (0,1) range");

            }

            //Check that the order of the filter is > 0
            if (order_filt <= 0)
            {

                throw new Exception("The order of the filter must be > 0");

            }

            if (!(save_filt_coeff == null))
            {

                //Clean up the global variables for a new analysis
                Array.Clear(save_filt_coeff, 0, save_filt_coeff.Length);
                Array.Clear(ptemp, 0, ptemp.Length);
                Array.Clear(p, 0, p.Length);
                a.Clear();
                Array.Clear(b, 0, b.Length);
                Array.Clear(c, 0, c.Length);
                Array.Clear(num_filt, 0, num_filt.Length);
                Array.Clear(den_filt, 0, den_filt.Length);

            }

            else
            {

                save_filt_coeff = new double[2][];

                for (int kk = 0; kk < 2; kk++)
                {

                    save_filt_coeff[kk] = new double[2 * order_filt + 1];

                }

            }

            int type_filt = 0;

            //Step 1: get analog, pre - warped frequencies
            Freq_pre_wrapped(type_filt, W_f1, W_f2);


            //Step 2: convert to low-pass prototype estimate
            Wn_f1_Wn_f2(type_filt, u_f1, u_f2);


            //Step 3: Get N - th order Butterworth analog lowpass prototype
            Buttap(order_filt);

            //Step 4: Transform to state-space
            Zp2ss(order_filt);


            if (order_filt > 1)
            {

                temp_dim_arr_matr -= 2;

            }

            else
            {

                temp_dim_arr_matr = order_filt;

            }


            //Copy the values of the matrix/arrays "arma" matrix/array in order to compute the pseudo-inverse of the matrix and other matrix operations
            a_arma = Matrix<Complex>.Build.Dense(2 * temp_dim_arr_matr, 2 * temp_dim_arr_matr);
            Matrix<Complex> a_arma_p_eye = Matrix<Complex>.Build.DenseIdentity(temp_dim_arr_matr, temp_dim_arr_matr);
            Matrix<Complex> a_arma_n_eye = -Matrix<Complex>.Build.DenseIdentity(temp_dim_arr_matr, temp_dim_arr_matr);
            b_arma = Matrix<Complex>.Build.Dense(2 * temp_dim_arr_matr, 1);
            c_arma = Matrix<Complex>.Build.Dense(1, 2 * temp_dim_arr_matr);
            d_arma = Matrix<Complex>.Build.Dense(1, 1);


            double q = Wn / Bw;
            for (int kk = 0; kk < 2 * temp_dim_arr_matr; kk++)
            {
                if (kk < temp_dim_arr_matr)
                {

                    b_arma[kk, 0] = b[kk] * Wn / q;
                    c_arma[0, kk] = c[kk];

                }

                for (int ll = 0; ll < 2 * temp_dim_arr_matr; ll++)
                {

                    if (kk < temp_dim_arr_matr)
                    {

                        if (ll < temp_dim_arr_matr)

                        {

                            a_arma[kk, ll] = Wn * a[kk, ll] / q;

                        }

                        else
                        {

                            a_arma[kk, ll] = Wn * a_arma_p_eye[kk, ll - temp_dim_arr_matr];

                        }
                    }

                    else
                    {
                        if (ll < temp_dim_arr_matr)
                        {

                            a_arma[kk, ll] = Wn * a_arma_n_eye[kk - temp_dim_arr_matr, ll];

                        }
                    }

                }

            }


            //Step 5: Use Bilinear transformation to find discrete equivalent
            Bilinear(a_arma, b_arma, c_arma, d_arma, fs, type_filt);

            //Step 6: Transform to zero-pole-gain and polynomial forms
            Zero_pole_gain(ad_arma, type_filt, order_filt, Wn, Bw);

            return save_filt_coeff;

        }

        //Estimate the coeffients of a band-stop filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        public double[][] Lp2bs(double W_f1, double W_f2, int order_filt)
        {

            //Check that the low cut-off frequency is higher than the low cut-off frequency
            if (W_f2 <= W_f1)
            {

                throw new Exception("The low cut-off frequency needs to be higher than the low cut-off frequency");

            }
            
            //Check that the normalized frequencies are within the correct range of values
            if (W_f1 <= 0 | W_f1 >= 1 | W_f2 <= 0 | W_f2 >= 1)
            {

                throw new Exception("Cut-off frequencies must be in the (0,1) range");

            }

            //Check that the order of the filter is > 0
            if (order_filt <= 0)
            {

                throw new Exception("The order of the filter must be > 0");

            }

            if (!(save_filt_coeff == null))
            {

                //Clean up the global variables for a new analysis
                Array.Clear(save_filt_coeff, 0, save_filt_coeff.Length);
                Array.Clear(ptemp, 0, ptemp.Length);
                Array.Clear(p, 0, p.Length);
                a.Clear();
                Array.Clear(b, 0, b.Length);
                Array.Clear(c, 0, c.Length);
                Array.Clear(num_filt, 0, num_filt.Length);
                Array.Clear(den_filt, 0, den_filt.Length);

            }

            else
            {

                save_filt_coeff = new double[2][];

                for (int kk = 0; kk < 2; kk++)
                {

                    save_filt_coeff[kk] = new double[2 * order_filt + 1];

                }

            }

            int type_filt = 1;

            //Step 1: get analog, pre - warped frequencies
            Freq_pre_wrapped(type_filt, W_f1, W_f2);

            //Step 2: convert to low-pass prototype estimate
            Wn_f1_Wn_f2(type_filt, u_f1, u_f2);

            //Step 3: Get N - th order Butterworth analog lowpass prototype
            Buttap(order_filt);

            //Step 4: Transform to state-space
            Zp2ss(order_filt);

            if (order_filt > 1)
            {

                temp_dim_arr_matr -= 2;

            }

            else
            {

                temp_dim_arr_matr = order_filt;

            }

            //Copy the values of the matrix/arrays "arma" matrix/array in order to compute the pseudo-inverse of the matrix and other matrix operations
            a_arma = Matrix<Complex>.Build.Dense(2 * temp_dim_arr_matr, 2 * temp_dim_arr_matr);
            Matrix<Complex> a_arma_p_eye = Matrix<Complex>.Build.DenseIdentity(temp_dim_arr_matr, temp_dim_arr_matr);
            Matrix<Complex> a_arma_n_eye = -Matrix<Complex>.Build.DenseIdentity(temp_dim_arr_matr, temp_dim_arr_matr);
            b_arma = Matrix<Complex>.Build.Dense(2 * temp_dim_arr_matr, 1);
            c_arma = Matrix<Complex>.Build.Dense(1, 2 * temp_dim_arr_matr);
            d_arma = Matrix<Complex>.Build.Dense(1, 1);


            Matrix<Complex> a_arma_pinv = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, temp_dim_arr_matr);
            Matrix<Complex> b_arma_temp = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, 1);
            Matrix<Complex> c_arma_temp = Matrix<Complex>.Build.Dense(1, temp_dim_arr_matr);

            Matrix<Complex> a_b_arma_temp = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, 1);
            Matrix<Complex> c_a_arma_temp = Matrix<Complex>.Build.Dense(1, temp_dim_arr_matr);

            for (int kk = 0; kk < temp_dim_arr_matr; kk++)
            {
                b_arma_temp[kk, 0] = b[kk];
                c_arma_temp[0, kk] = c[kk];

                for (int ll = 0; ll < temp_dim_arr_matr; ll++)
                {

                    a_arma_pinv[kk, ll] = a[kk, ll];

                }

            }

            double q = Wn / Bw;

            a_arma_pinv = (Wn / q) * (a_arma_pinv.PseudoInverse());

            a_b_arma_temp = (Wn) * (a_arma_pinv * b_arma_temp) / q;
            c_a_arma_temp = c_arma_temp * a_arma_pinv;

            for (int kk = 0; kk < 2 * temp_dim_arr_matr; kk++)
            {
                if (kk < temp_dim_arr_matr)
                {

                    b_arma[kk, 0] = -a_b_arma_temp[kk, 0];
                    c_arma[0, kk] = c_a_arma_temp[0, kk];

                }

                for (int ll = 0; ll < 2 * temp_dim_arr_matr; ll++)
                {

                    if (kk < temp_dim_arr_matr)
                    {

                        if (ll < temp_dim_arr_matr)

                        {

                            a_arma[kk, ll] = a_arma_pinv[kk, ll];

                        }

                        else
                        {

                            a_arma[kk, ll] = Wn * a_arma_p_eye[kk, ll - temp_dim_arr_matr];

                        }
                    }

                    else
                    {
                        if (ll < temp_dim_arr_matr)
                        {

                            a_arma[kk, ll] = Wn * a_arma_n_eye[kk - temp_dim_arr_matr, ll];

                        }
                    }

                }

            }

            d_arma = d + c_a_arma_temp * b_arma_temp;

            //Step 5: Use Bilinear transformation to find discrete equivalent
            Bilinear(a_arma, b_arma, c_arma, d_arma, fs, type_filt);

            //Step 6: Transform to zero-pole-gain and polynomial forms
            Zero_pole_gain(ad_arma, type_filt, order_filt, Wn, Bw);

            return save_filt_coeff;

        }

        //Estimate the coeffients of a high-pass filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        public double[][] Lp2hp(double W_f1, int order_filt)
        {

            //Check that the normalized frequencies are within the correct range of values
            if (W_f1 <= 0 | W_f1 >= 1)
            {

                throw new Exception("Cut-off frequencies must be in the (0,1) range");

            }

            //Check that the order of the filter is > 0
            if (order_filt <= 0)
            {

                throw new Exception("The order of the filter must be > 0");

            }


            if (!(save_filt_coeff == null))
            {

                //Clean up the global variables for a new analysis
                Array.Clear(save_filt_coeff, 0, save_filt_coeff.Length);
                Array.Clear(ptemp, 0, ptemp.Length);
                Array.Clear(p, 0, p.Length);
                a.Clear();
                Array.Clear(b, 0, b.Length);
                Array.Clear(c, 0, c.Length);
                Array.Clear(num_filt, 0, num_filt.Length);
                Array.Clear(den_filt, 0, den_filt.Length);

            }

            else
            {

                save_filt_coeff = new double[2][];

                for (int kk = 0; kk < 2; kk++)
                {

                    save_filt_coeff[kk] = new double[order_filt + 1];

                }

            }

            int type_filt = 3;

            //Step 1: get analog, pre - warped frequencies
            Freq_pre_wrapped(type_filt, W_f1, 0);

            //Step 2: convert to low-pass prototype estimate
            Wn_f1_Wn_f2(type_filt, u_f1, u_f2);

            //Step 3: Get N - th order Butterworth analog lowpass prototype
            Buttap(order_filt);

            //Step 4: Transform to state-space
            Zp2ss(order_filt);

            if (order_filt > 1)
            {

                temp_dim_arr_matr -= 2;

            }

            else
            {

                temp_dim_arr_matr = order_filt;

            }

            //Copy the values of the matrix/arrays "arma" matrix/array in order to compute the pseudo-inverse of the matrix and other matrix operations
            a_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, temp_dim_arr_matr);
            b_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, 1);
            c_arma = Matrix<Complex>.Build.Dense(1, temp_dim_arr_matr);
            d_arma = Matrix<Complex>.Build.Dense(1, 1);

            for (int kk = 0; kk < temp_dim_arr_matr; kk++)
            {
                b_arma[kk, 0] = b[kk];
                c_arma[0, kk] = c[kk];

                for (int ll = 0; ll < temp_dim_arr_matr; ll++)
                {

                    a_arma[kk, ll] = a[kk, ll];

                }

            }

            d_arma = d_arma - (c_arma * (a_arma.PseudoInverse())) * b_arma;
            c_arma = c_arma * (a_arma.PseudoInverse());
            b_arma = -Wn * (a_arma.PseudoInverse()) * b_arma;
            a_arma = Wn * (a_arma.PseudoInverse());


            //Step 5: Use Bilinear transformation to find discrete equivalent
            Bilinear(a_arma, b_arma, c_arma, d_arma, fs, type_filt);

            //Step 6: Transform to zero-pole-gain and polynomial forms
            Zero_pole_gain(ad_arma, type_filt, order_filt, Wn, Bw);

            return save_filt_coeff;


        }


        //Estimate the coeffients of a low-pass filter and return a 2 rows x N coefficients matrix. Row 1 = Numerator; Row 2 = Denumerator
        public double[][] Lp2lp(double W_f2, int order_filt)
        {

            //Check that the normalized frequencies are within the correct range of values
            if (W_f2 <= 0 | W_f2 >= 1)
            {

                throw new Exception("Cut-off frequencies must be in the (0,1) range");

            }

            //Check that the order of the filter is > 0
            if (order_filt <= 0)
            {

                throw new Exception("The order of the filter must be > 0");

            }

            if (!(save_filt_coeff == null))
            {

                //Clean up the global variables for a new analysis
                Array.Clear(save_filt_coeff, 0, save_filt_coeff.Length);
                Array.Clear(ptemp, 0, ptemp.Length);
                Array.Clear(p, 0, p.Length);
                a.Clear();
                Array.Clear(b, 0, b.Length);
                Array.Clear(c, 0, c.Length);
                Array.Clear(num_filt, 0, num_filt.Length);
                Array.Clear(den_filt, 0, den_filt.Length);

            }

            else
            {

                save_filt_coeff = new double[2][];

                for (int kk = 0; kk < 2; kk++)
                {

                    save_filt_coeff[kk] = new double[order_filt + 1];

                }

            }

            int type_filt = 2;

            //Step 1: get analog, pre - warped frequencies
            Freq_pre_wrapped(type_filt, 0, W_f2);

            //Step 2: convert to low-pass prototype estimate
            Wn_f1_Wn_f2(type_filt, u_f1, u_f2);

            //Step 3: Get N - th order Butterworth analog lowpass prototype
            Buttap(order_filt);

            //Step 4: Transform to state-space
            Zp2ss(order_filt);

            if (order_filt > 1)
            {

                temp_dim_arr_matr -= 2;

            }

            else
            {

                temp_dim_arr_matr = order_filt;

            }

            //Copy the values of the matrix/arrays "arma" matrix/array in order to compute the pseudo-inverse of the matrix and other matrix operations
            a_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, temp_dim_arr_matr);
            b_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, 1);
            c_arma = Matrix<Complex>.Build.Dense(1, temp_dim_arr_matr);
            d_arma = Matrix<Complex>.Build.Dense(1, 1);

            for (int kk = 0; kk < temp_dim_arr_matr; kk++)
            {
                b_arma[kk, 0] = b[kk];
                c_arma[0, kk] = c[kk];

                for (int ll = 0; ll < temp_dim_arr_matr; ll++)
                {

                    a_arma[kk, ll] = a[kk, ll];

                }

            }


            b_arma = Wn * b_arma;
            a_arma = Wn * a_arma;


            //Step 5: Use Bilinear transformation to find discrete equivalent
            Bilinear(a_arma, b_arma, c_arma, d_arma, fs, type_filt);

            //Step 6: Transform to zero-pole-gain and polynomial forms
            Zero_pole_gain(ad_arma, type_filt, order_filt, Wn, Bw);

            return save_filt_coeff;

        }


        //Step 5: Use Bilinear transformation to find discrete equivalent
        public void Bilinear(Matrix<Complex> a_arma_f, Matrix<Complex> b_arma_f, Matrix<Complex> c_arma_f, Matrix<Complex> d_arma_f, double fs_f, int type_filt_f)
        {
            double t_arma;
            double r_arma;

            Matrix<Complex> t1_arma_eye;
            Matrix<Complex> t2_arma_eye;

            if (type_filt_f > 1)
            {
                t1_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, temp_dim_arr_matr);
                t1_arma_eye = Matrix<Complex>.Build.DenseIdentity(temp_dim_arr_matr, temp_dim_arr_matr);
                t2_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, temp_dim_arr_matr);
                t2_arma_eye = Matrix<Complex>.Build.DenseIdentity(temp_dim_arr_matr, temp_dim_arr_matr);
                ad_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, temp_dim_arr_matr);
                bd_arma = Matrix<Complex>.Build.Dense(temp_dim_arr_matr, 1);
                cd_arma = Matrix<Complex>.Build.Dense(1, temp_dim_arr_matr);
                dd_arma = Matrix<Complex>.Build.Dense(1, 1);
            }

            else
            {

                t1_arma = Matrix<Complex>.Build.DenseIdentity(2 * temp_dim_arr_matr, 2 * temp_dim_arr_matr);
                t1_arma_eye = Matrix<Complex>.Build.DenseIdentity(2 * temp_dim_arr_matr, 2 * temp_dim_arr_matr);
                t2_arma = Matrix<Complex>.Build.DenseIdentity(2 * temp_dim_arr_matr, 2 * temp_dim_arr_matr);
                t2_arma_eye = Matrix<Complex>.Build.DenseIdentity(2 * temp_dim_arr_matr, 2 * temp_dim_arr_matr);
                ad_arma = Matrix<Complex>.Build.Dense(2 * temp_dim_arr_matr, 2 * temp_dim_arr_matr);
                bd_arma = Matrix<Complex>.Build.Dense(2 * temp_dim_arr_matr, 1);
                cd_arma = Matrix<Complex>.Build.Dense(1, 2 * temp_dim_arr_matr);
                dd_arma = Matrix<Complex>.Build.Dense(1, 1);

            }

            t_arma = (1 / fs_f);
            r_arma = Math.Sqrt(t_arma);
            t1_arma = t1_arma_eye + a_arma_f * t_arma * 0.5; //t1_arma.eye() 
            t2_arma = t2_arma_eye - a_arma_f * t_arma * 0.5;
            ad_arma = t1_arma * (t2_arma.PseudoInverse());
            bd_arma = (t_arma / r_arma) * t2_arma.Solve(b_arma_f);//arma::solve(t2_arma, b_arma_f);
            cd_arma = (r_arma * c_arma_f) * (t2_arma.PseudoInverse());
            dd_arma = (c_arma_f * t2_arma.PseudoInverse()) * b_arma_f * (t_arma / 2) + d_arma_f;

        }

        //Step 6: Transform to zero-pole-gain and polynomial forms
        public void Zero_pole_gain(Matrix<Complex> a_arma_f, int type_filt_f, int order_filt_f, double Wn_f_f, double Bw_f)
        {
            int dim_array;

            if (type_filt_f > 1)
            {

                //Initialize the vectors "num_filt" and "den_filt"
                num_filt = new double[order_filt_f + 1];
                den_filt = new double[order_filt_f + 1];

                dim_array = temp_dim_arr_matr;

            }

            else
            {

                //Initialize the vectors "num_filt" and "den_filt"
                num_filt = new double[2 * order_filt_f + 1];
                den_filt = new double[2 * order_filt_f + 1];

                dim_array = 2 * temp_dim_arr_matr;

            }

            //Extract the coefficients of the denumerator
            Complex[] coeff_pol = new Complex[temp_dim_arr_matr + 1];

            if (type_filt_f > 1)

            {

                coeff_pol = Char_poly(a_arma_f, temp_dim_arr_matr);

            }

            else
            {

                coeff_pol = Char_poly(a_arma_f, 2 * temp_dim_arr_matr);

            }


            for (int qq = 0; qq < dim_array + 1; qq++)
            {

                den_filt[qq] = coeff_pol[qq].Real;
                save_filt_coeff[1][qq] = den_filt[qq];

            }


            //Extract the coefficients of the denominator
            double w = 0;
            Wn = 2 * Math.Atan2(Wn, 4);
            Complex[] r;

            switch (type_filt_f)
            {

                case 0: // band-pass


                    r = new Complex[dim_array + 1];

                    for (int kk = 0; kk < dim_array; kk++)
                    {

                        if (kk < temp_dim_arr_matr)
                        {

                            r[kk] = 1;

                        }

                        else
                        {

                            r[kk] = -1;

                        }

                    }

                    w = Wn;

                    break;

                case 1: // band-stop

                    r = new Complex[dim_array + 1];


                    for (int kk = 0; kk < dim_array; kk++)
                    {

                        r[kk] = Complex.Exp(complex_imag * Wn * Math.Pow(-1, kk));

                    }
                    w = 0;

                    break;

                case 2: // low-pass

                    r = new Complex[dim_array + 1];

                    for (int kk = 0; kk < dim_array; kk++)
                    {

                        r[kk] = -1;

                    }
                    w = 0;
                    break;

                case 3: //high-pass

                    r = new Complex[dim_array + 1];

                    for (int kk = 0; kk < dim_array; kk++)
                    {

                        r[kk] = 1;

                    }

                    w = Math.PI;
                    break;

                default:

                    r = new Complex[dim_array + 1];
                    break;

            }

            Complex[] coeff_pol_num = new Complex[dim_array + 1];

            coeff_pol_num = Poly(r, dim_array);

            Complex[] kern = new Complex[dim_array + 1];

            for (int kk = 0; kk < dim_array + 1; kk++)
            {

                kern[kk] = Complex.Exp(-complex_imag * w * kk);

            }

            Complex temp_sum_I;
            Complex temp_sum_II;

            for (int kk = 0; kk < dim_array + 1; kk++)
            {

                temp_sum_I = new Complex(0.0, 0.0);
                temp_sum_II = new Complex(0.0, 0.0);

                for (int hh = 0; hh < dim_array + 1; hh++)
                {

                    temp_sum_I += kern[hh] * den_filt[hh];
                    temp_sum_II += kern[hh] * coeff_pol_num[hh];

                }

                num_filt[kk] = (coeff_pol_num[kk] * temp_sum_I / temp_sum_II).Real;
                save_filt_coeff[0][kk] = num_filt[kk];

            }

        }

        public bool Check_stability_iir(double[][] coeff_filt)
        {
            bool stability_flag = true;


            //Coefficients need to be organized in ascending order
            double[] temp_coeff_den = new double[coeff_filt[1].Length];
            for (int kk = 0; kk < coeff_filt[1].Length; kk++)
            {

                temp_coeff_den[kk] = coeff_filt[1][coeff_filt[1].Length - 1 - kk];

            }

            Complex[] roots_den = FindRoots.Polynomial(temp_coeff_den);

            double[] magnitude_roots_den = new double[roots_den.Length];

            for (int kk = 0; kk < roots_den.Length; kk++)
            {

                magnitude_roots_den[kk] = Complex.Abs(roots_den[kk]);

                if (magnitude_roots_den[kk] >= 1)
                {

                    stability_flag = false;
                    break;
                }

            }

            return stability_flag;

        }

        //Filter the data by using the Direct-Form II Transpose, as explained in the Matlab documentation
        public double[] Filter_Data(double[][] coeff_filt, double[] pre_filt_signal)
        {

            double[] filt_signal = new double[pre_filt_signal.Length];
            Array.Clear(filt_signal, 0, filt_signal.Length);

            double[][] w_val = new double[coeff_filt[0].Length][];


            for (int ff = 0; ff < coeff_filt[0].Length; ff++)
            {

                w_val[ff] = new double[pre_filt_signal.Length];

            }


            //Convolution product to filter the data
            for (int kk = 0; kk < pre_filt_signal.Length; kk++)
            {

                if (kk == 0)
                {

                    filt_signal[kk] = pre_filt_signal[kk] * coeff_filt[0][0];


                    for (int ww = 1; ww < coeff_filt[0].Length; ww++)
                    {

                        w_val[ww - 1][kk] = pre_filt_signal[kk] * coeff_filt[0][ww] - filt_signal[kk] * coeff_filt[1][ww];


                    }

                }

                else
                {

                    filt_signal[kk] = pre_filt_signal[kk] * coeff_filt[0][0] + w_val[0][kk - 1];

                    for (int ww = 1; ww < coeff_filt[0].Length; ww++)
                    {

                        w_val[ww - 1][kk] = pre_filt_signal[kk] * coeff_filt[0][ww] + w_val[ww][kk - 1] - filt_signal[kk] * coeff_filt[1][ww];

                        if (ww == coeff_filt[0].Length - 1)
                        {

                            w_val[ww - 1][kk] = pre_filt_signal[kk] * coeff_filt[0][ww] - filt_signal[kk] * coeff_filt[1][ww];

                        }

                    }

                }



            }

            return filt_signal;

        }
    }
}
