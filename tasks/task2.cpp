#include <vector>
#include <string>

#include <igl/readOBJ.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include "imgui.h"




void task2() {

    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    std::string robot_dir = "robot/franka_description";
    std::string urdf_path = robot_dir + "/robot.urdf";

    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;

    // TODO 1: Build the robot model from the URDF file
    //  1). Use pinocchio::urdf::buildModel() to construct the kinematic model.
    //  2). Use pinocchio::urdf::buildGeom() to load the visual geometry.

    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::VISUAL, geom_model, robot_dir);


    pinocchio::Data data(model);
    pinocchio::GeometryData geom_data(geom_model);

    Eigen::VectorXd q = pinocchio::neutral(model);


    for (size_t i = 0; i < geom_model.geometryObjects.size(); ++i) {
        const auto& obj = geom_model.geometryObjects[i];

        // TODO 2: Register all meshes of the robot
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::readOBJ(obj.meshPath, V, F);
        polyscope::registerSurfaceMesh(obj.meshPath,V, F);



    }



    polyscope::show();
}