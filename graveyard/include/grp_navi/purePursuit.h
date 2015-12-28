#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <grp_navi/control.h>

using namespace Eigen;

class purePursuit{
public:
    purePursuit();

    control get_control(Vector3d robot_pose, Vector2d goal);

private:
    control ctrl;
};
