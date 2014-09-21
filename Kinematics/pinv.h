#ifndef __pinv_h__
#define __pinv_h__

#include <Eigen/Core>
#include <Eigen/SVD>

/*
Returns the Moore-Penrose peudoinverse of a given matrix M.
Adapted from: http://eigen.tuxfamily.org/index.php?title=FAQ#Is_there_a_method_to_compute_the_.28Moore-Penrose.29_pseudo_inverse_.3F
*/
template< typename MatrixType >
MatrixType pinv( MatrixType& M, double epsilon = 1e-6 )
{
    Eigen::JacobiSVD< MatrixType > svd( M, Eigen::ComputeThinU | Eigen::ComputeThinV );
    typedef typename Eigen::JacobiSVD< MatrixType >::SingularValuesType SingularValuesType;
    const SingularValuesType& singularValues = svd.singularValues();
    SingularValuesType singularValues_inv = singularValues;
    for( long i=0; i < singularValues.rows(); ++i )
    {
        if( singularValues(i) > epsilon )
        {
            singularValues_inv(i)=1.0/singularValues(i);
        }
        else
        {
            singularValues_inv(i)=0;
        }
    }
    return svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose();
}

#endif /* __pinv_h__ */
