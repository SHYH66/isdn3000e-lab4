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




void task3() {

    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    std::string robot_dir = "robot/franka_description";
    std::string urdf_path = robot_dir + "/robot.urdf";

    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;

    // Same as Task2: build model + geometry
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::VISUAL, geom_model, robot_dir);

    pinocchio::Data data(model);
    pinocchio::GeometryData geom_data(geom_model);

    Eigen::VectorXd q = pinocchio::neutral(model);
    float q_ui[7] = {0,0,0,0,0,0,0};

    const float q_min[7] = {-2.9f, -1.8f, -2.9f, -3.1f, -2.9f, -0.1f, -2.9f};
    const float q_max[7] = { 2.9f,  1.8f,  2.9f,  0.1f,  2.9f,  3.7f,  2.9f};

    std::vector<Eigen::MatrixXd> V_locals;
    std::vector<polyscope::SurfaceMesh*> meshes;

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);

    for (int i = 0; i < geom_model.geometryObjects.size(); ++i) {
        const auto& obj = geom_model.geometryObjects[i];

        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::readOBJ(obj.meshPath, V, F);

        pinocchio::SE3 M = geom_data.oMg[i];
        Eigen::Matrix3d R = M.rotation();
        Eigen::Vector3d t = M.translation();
        Eigen::MatrixXd Vw = (V * R.transpose()).rowwise() + t.transpose();

        auto* ms = polyscope::registerSurfaceMesh(obj.name + "_" + std::to_string(i), Vw, F);

        V_locals.push_back(V);
        meshes.push_back(ms);
    }

    polyscope::state::userCallback = [&]() {
        ImGui::Text("Task3: Joint sliders");
        ImGui::Separator();

        // TODO 1: Create 7 sliders for 7 joints, whose value should be between q_min and q_max.
        //  The sliders update the q_ui instead of q.

        for (int j = 0; j < 7; ++j) {
            std::string label = "q" + std::to_string(j);
            ImGui::SliderFloat(label.c_str(), &q_ui[j], q_min[j], q_max[j]);
        }


        for (int j = 0; j < 7; ++j)
        {
            q[j] = q_ui[j];
        }



        // TODO 2: Use the slider values to update robot pose + meshes
        //  1) Recompute FK + frame placements + geometry placements.
        //  2) For each mesh, use geom_data.oMg[i] to compute the new world vertices,
        //    then call meshes[i]->updateVertexPositions().
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);

        for (int i = 0; i < meshes.size(); ++i)
        {
            pinocchio::SE3 M = geom_data.oMg[i];
            Eigen::Matrix3d R = M.rotation();
            Eigen::Vector3d t = M.translation();

            Eigen::MatrixXd Vw = (V_locals[i] * R.transpose()).rowwise() + t.transpose();
            meshes[i]->updateVertexPositions(Vw);
        }




    };

    polyscope::show();
}