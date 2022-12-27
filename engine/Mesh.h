#pragma once

#include <Eigen/Core>
#include <iostream>
#include <utility>
#include <vector>
#include <igl/min_heap.h>


namespace cg3d
{

    struct MeshData
    {
        const Eigen::MatrixXd vertices; // Vertices of the mesh (#V x 3)
        const Eigen::MatrixXi faces; // Faces of the mesh (#F x 3)
        const Eigen::MatrixXd vertexNormals; // One normal per vertex
        const Eigen::MatrixXd textureCoords; // UV vertices
    };

    class Mesh
    {
    public:
        std::string name;

        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        Eigen::MatrixXi E;
        Eigen::VectorXi EMAP;
        Eigen::MatrixXi EF, EI;
        Eigen::VectorXi EQ;
        Eigen::MatrixXd C;

        igl::min_heap<std::tuple<double, int, int>> Q;

        void init_my_data();
        int simplify(int edges_to_collapse);

        std::vector<MeshData> data;

        Mesh(std::string name, Eigen::MatrixXd vertices, Eigen::MatrixXi faces, Eigen::MatrixXd vertexNormals, Eigen::MatrixXd textureCoords);
        Mesh(std::string name, std::vector<MeshData> data) : name(std::move(name)), data(std::move(data)) {

            init_my_data();
        };

        Mesh(const Mesh& mesh) = default;

        static const std::shared_ptr<Mesh>& Plane();
        static const std::shared_ptr<Mesh>& Cube();
        static const std::shared_ptr<Mesh>& Tetrahedron();
        static const std::shared_ptr<Mesh>& Octahedron();
        static const std::shared_ptr<Mesh>& Cylinder();
    };

} // namespace cg3d