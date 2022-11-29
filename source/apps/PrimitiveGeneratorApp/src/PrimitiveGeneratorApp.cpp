#include "PrimitiveGeneratorApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/Renderer.h>

namespace lenny {

PrimitiveGeneratorApp::PrimitiveGeneratorApp() : gui::Application("PrimitiveGeneratorApp") {
    showOrigin = false;
}

void PrimitiveGeneratorApp::autogenerateCollisionPrimitives() {
    //Clear primitives
    agent.collisionPrimitives.clear();

    //Loop over all links
    for (const auto& [linkName, link] : agent.robot.links) {
        for (const robot::Visual& visual : link.visuals) {
            //Cast into gui model
            if (const gui::Model* const model = dynamic_cast<gui::Model*>(visual.model.get())) {
                //Generate bounding box info
                enum POINTS { X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX };
                static const std::array<uint, 6> index = {0, 0, 1, 1, 2, 2};
                static const std::array<double, 6> signs = {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
                std::array<double, 6> minMax;
                for (int i = 0; i < 6; i++)
                    minMax.at(i) = -signs.at(i) * HUGE_VALF;

                //Loop over meshes
                for (const gui::Model::Mesh& mesh : model->meshes) {
                    //Loop over vertices
                    for (const gui::Model::Mesh::Vertex& vertex : mesh.vertices) {
                        const Eigen::Vector3d vertexPosition = gui::utils::toEigen(vertex.position).cwiseProduct(visual.scale);
                        const Eigen::Vector3d localPosition = visual.localTrafo.getGlobalCoordinatesForPoint(vertexPosition);
                        const Eigen::Vector3d globalPosition = agent.computeGlobalPoint(agent.getInitialAgentState(), localPosition, linkName);

                        //Compute min-max points
                        for (int i = 0; i < 6; i++)
                            if (signs.at(i) * globalPosition[index.at(i)] > signs.at(i) * minMax.at(i))
                                minMax.at(i) = globalPosition[index.at(i)];
                    }
                }

                //Compute dimensions of bounding box
                const Eigen::Vector3d dimension(minMax.at(X_MAX) - minMax.at(X_MIN), minMax.at(Y_MAX) - minMax.at(Y_MIN), minMax.at(Z_MAX) - minMax.at(Z_MIN));
                uint maxDim = 0, minDim = 0, otherDim = 0;
                for (uint i = 1; i < 3; i++) {
                    if (dimension[i] > dimension[maxDim])
                        maxDim = i;
                    else if (dimension[i] < dimension[minDim])
                        minDim = i;
                    else
                        otherDim = i;
                }

                //Compute center point
                const Eigen::Vector3d globalCenterPoint =
                    0.5 * Eigen::Vector3d(minMax.at(X_MAX) + minMax.at(X_MIN), minMax.at(Y_MAX) + minMax.at(Y_MIN), minMax.at(Z_MAX) + minMax.at(Z_MIN));

                //Generate primitives
                if (fabs(dimension.x() - dimension.y()) < sphereTolerance && fabs(dimension.x() - dimension.z()) < sphereTolerance &&
                    fabs(dimension.y() - dimension.z()) < sphereTolerance) {
                    //--- Create a sphere
                    const double radius = dimFactor * 0.5 * dimension.norm();
                    agent.addCollisionSphere(linkName, agent.computeLocalPoint(agent.getInitialAgentState(), globalCenterPoint, linkName), radius);
                } else if (fabs(dimension[otherDim] - dimension[minDim]) < capsuleTolerance) {
                    //--- Create a capsule
                    const double radius = dimFactor * 0.5 * sqrt(dimension[minDim] * dimension[minDim] + dimension[otherDim] * dimension[otherDim]);
                    Eigen::Vector3d globalPoint1(globalCenterPoint), globalPoint2(globalCenterPoint);
                    globalPoint1[maxDim] += 0.5 * (dimension[maxDim] - radius / dimFactor);
                    globalPoint2[maxDim] -= 0.5 * (dimension[maxDim] - radius / dimFactor);
                    agent.addCollisionCapsule(linkName, agent.computeLocalPoint(agent.getInitialAgentState(), globalPoint1, linkName),
                                              agent.computeLocalPoint(agent.getInitialAgentState(), globalPoint2, linkName), radius);
                } else {
                    //--- Create a box
                    agent.addCollisionBox(linkName, agent.computeLocalPoint(agent.getInitialAgentState(), globalCenterPoint, linkName),
                                          agent.computeLocalOrientation(agent.getInitialAgentState(), Eigen::QuaternionD::Identity(), linkName),
                                          dimFactor * dimension, 0.01);
                }
            }
        }
    }
}

void PrimitiveGeneratorApp::process() {
    autogenerateCollisionPrimitives();
}

void PrimitiveGeneratorApp::drawScene() const {
    agent.drawScene(agent.getInitialAgentState());
}

void PrimitiveGeneratorApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");
    agent.drawGui(true);

    if (ImGui::TreeNode("App Settings")) {
        ImGui::InputDouble("Sphere Tolerance", &sphereTolerance);
        ImGui::InputDouble("Capsule Tolerance", &capsuleTolerance);
        ImGui::InputDouble("Dim Factor", &dimFactor);

        ImGui::TreePop();
    }

    ImGui::End();
}

}  // namespace lenny