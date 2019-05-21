#include "ur5_MR_description.h"

static const double W1 = 0.10915;
static const double W2 = 0.0823;
static const double L1 = 0.42500;
static const double L2 = 0.39225;
static const double H1 = 0.089159;
static const double H2 = 0.09465;

Eigen::MatrixXd get_Slist()
{
    Eigen::MatrixXd Slist(6,6);
    Slist << 0,   0,   0,     0,     0,     0,
             0,   1,   1,     1,     0,     1,
             1,   0,   0,     0,    -1,     0,
             0, -H1, -H1,   -H1,   -W1, H2-H1,
             0,   0,   0,     0, L1+L2,     0,
             0,   0,  L1, L1+L2,     0, L1+L2;

    return Slist;
}

Eigen::MatrixXd get_Mlist()
{
    Eigen::MatrixXd M(4,4);
    M << -1, 0, 0, L1+L2,
          0, 0, 1, W1+W2,
          0, 1, 0, H1-H2,
          0, 0, 0,     1;

    return M;
}
