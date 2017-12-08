// Example By: RandomVibe
// Eigen Doc: http://eigen.tuxfamily.org/dox/
// Quick Reference: http://eigen.tuxfamily.org/dox/QuickRefPage.html

#include <Eigen.h>     // Calls main Eigen matrix class library
#include <Eigen/LU>             // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices
#include <Streaming.h>

void print_mtxf(const Eigen::MatrixXd& X);

void setup() {

   Serial.begin(115200);
   Serial.println();
   Serial.println();
   delay(1000);
   
   // DECLARE MATRICES 
   //--------------------
   MatrixXd Pp(6,6);   // Produces 6x6 double matrix class
   MatrixXd H(6,6);    // Note: without "using namespace Eigen", declaration would be: Eigen::MatrixXf H(6,6);
   MatrixXd R(6,6);  
   MatrixXd X(6,6);  
   MatrixXd K(6,6);  
   MatrixXd Z(6,6);  

   // INPUT MATRICES (so-called comma-initialize syntax)
   //---------------------------------------------------------
   Pp << 0.3252,  0.3192,  1.0933, -0.0068, -1.0891, -1.4916,
        -0.7549,  0.3129,  1.1093,  1.5326,  0.0326, -0.7423,
         1.3703, -0.8649, -0.8637, -0.7697,  0.5525, -1.0616,
        -1.7115, -0.0301,  0.0774,  0.3714,  1.1006,  2.3505,
        -0.1022, -0.1649, -1.2141, -0.2256,  1.5442, -0.6156,
        -0.2414,  0.6277, -1.1135,  1.1174,  0.0859,  0.7481 ;

   H << 0.8147, 0.2785, 0.9572, 0.7922, 0.6787, 0.7060,
        0.9058, 0.5469, 0.4854, 0.9595, 0.7577, 0.0318,
        0.1270, 0.9575, 0.8003, 0.6557, 0.7431, 0.2769,
        0.9134, 0.9649, 0.1419, 0.0357, 0.3922, 0.0462,
        0.6324, 0.1576, 0.4218, 0.8491, 0.6555, 0.0971,
        0.0975, 0.9706, 0.9157, 0.9340, 0.1712, 0.8235;

   R << 0.3252,  0.3192,  1.0933, -0.0068, -1.0891, -1.4916,
       -0.7549,  0.3129,  1.1093,  1.5326,  0.0326, -0.7423,
        1.3703, -0.8649, -0.8637, -0.7697,  0.5525, -1.0616,
       -1.7115, -0.0301,  0.0774,  0.3714,  1.1006,  2.3505,
       -0.1022, -0.1649, -1.2141, -0.2256,  1.5442, -0.6156,
       -0.2414,  0.6277, -1.1135,  1.1174,  0.0859,  0.7481;


   // Kalman Gain Example; Matlab form:  K = Pp * H' * inv(H * Pp * H' + R)
   //-----------------------------------
   X  = H * Pp * H.transpose() + R;    
   K  = Pp * H.transpose() * X.inverse();   


   // Print Result
   //----------------------------
    print_mtxd(K);      // Print Matrix Result (passed by reference)

  // center matrix
  MatrixXd centered = K.rowwise() - K.colwise().mean();
  // compute covariance matrix
  MatrixXd cov = (centered.adjoint() * centered) / double(K.rows() - 1);


   print_mtxd(cov);      // Print Matrix Result (passed by reference)

  const byte i = 3;
  const byte j = 2;
  double test[i][j] = {
    {1,2},
    {3,4},
    {5,6}
  };
  
  Map<MatrixXd> mf((double*)&test, j, i);

  print_mtxd(mf);

}




void loop() {
 // put your main code here, to run repeatedly: 
 
}




// PRINT MATRIX (double type)
// By: randomvibe
//-----------------------------
void print_mtxd(const Eigen::MatrixXd& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}