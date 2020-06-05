// #include <boost/geometry.hpp>
// #include <boost/geometry/geometries/point_xy.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
#include <igl/readSTL.h>
// #include <igl/opengl/glfw/Viewer.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/AABB.h>

#include <iostream>

// namespace bg = boost::geometry;
using namespace std;

// typedef bg::model::point<double, 3, bg::cs::cartesian> point_t;
// typedef bg::model::polygon<point_t> polygon_t;
// typedef bg::model::multi_polygon<polygon_t> multi_polygon_t;

int main(int argc, char** argv)
{
    // polygon_t poly{ { { 0.0, 0.0, 0.0 }, { 1.0, 1.0, 0.0 }, { 2.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } } };
    // multi_polygon_t mesh{
    //     { { { 0.0, 0.0, 0.0 }, { 0.0, 5.0, 0.0 }, { 5.0, 5.0, 0.0 }, { 5.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } },
    //       { { 1.0, 1.0, 0.0 }, { 4.0, 1.0, 0.0 }, { 4.0, 4.0, 0.0 }, { 1.0, 4.0, 0.0 }, { 1.0, 1.0, 0.0 } },
    //       { { 5.0, 5.0, 0.0 }, { 5.0, 6.0, 0.0 }, { 6.0, 6.0, 0.0 }, { 6.0, 5.0, 0.0 }, { 5.0, 5.0, 0.0 } } }
    // };

    // point_t qp{ 1.0, 0.5, 100.0 };
    // cout << bg::dsv(qp) << endl << bg::dsv(poly) << endl << bg::dsv(mesh) << endl;
    // cout << "Point to multi_polygon: " << bg::comparable_distance(qp, mesh) << endl;
    // cout << "Point to polygon: " << bg::comparable_distance(qp, poly) << endl;

    // Load
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXi N;
    cout << "Loading\n";
    igl::readSTL("/home/user/catkin_ws/src/srcp2/data/lunar_terrain.stl", V, F, N);
    cout << "loaded" << endl;

    // Find clossest point
    Eigen::MatrixXd P(1, 3);
    Eigen::VectorXd sqrD;
    Eigen::VectorXi I;
    Eigen::MatrixXd C;
    igl::AABB<Eigen::MatrixXd, 3> tree;
    cout << "tree" << endl;
    tree.init(V, F);
    cout << "distance 1" << endl;
    P.block<1, 3>(0, 0) = Eigen::Vector3d(0, 0, 0);
    tree.squared_distance(V, F, P, sqrD, I, C);
    cout << "P: " << P << endl;
    cout << "C: " << C << endl;
    cout << "sqrD: " << sqrD << endl;

    cout << "distance 2" << endl;
    P.block<1, 3>(0, 0) = Eigen::Vector3d(10, 10, 10);
    tree.squared_distance(V, F, P, sqrD, I, C);
    cout << "P: " << P << endl;
    cout << "C: " << C << endl;
    cout << "sqrD: " << sqrD << endl;

    // igl::opengl::glfw::Viewer viewer;
    // viewer.data().set_mesh(V, F);
    // viewer.launch();

    return 0;
}
