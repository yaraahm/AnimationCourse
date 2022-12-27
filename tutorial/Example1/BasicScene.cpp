#include "BasicScene.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/edge_flaps.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) };

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/bunny.off") };

    sphere1 = Model::Create("sphere", sphereMesh, material);
    sphere1->Scale(25);
    sphere1->showWireframe = true;
    sphere1->Translate({ 0,-1,0 });

    camera->Translate(20, Axis::Z);
    root->AddChild(sphere1);


}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //sphere1->Rotate(0.001f, Axis::X);
}



void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            if (pickedModel != nullptr) {
                pickedModel->meshIndex++;
                if (pickedModel->meshIndex > pickedModel->GetMesh()->data.size())
                    pickedModel->meshIndex--;
                else
                    pickedModel->SetMeshList(pickedModel->GetMeshList());
            }

            break;
        case GLFW_KEY_DOWN:
            if (pickedModel != nullptr) {
                pickedModel->meshIndex--;
                if (pickedModel->meshIndex < 0)
                    pickedModel->meshIndex = 0;
                else
                    pickedModel->SetMeshList(pickedModel->GetMeshList());
            }
            break;
        case GLFW_KEY_LEFT:
            camera->RotateInSystem(system, 0.1f, Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            camera->RotateInSystem(system, -0.1f, Axis::Y);
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.05f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.05f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.05f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.05f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.05f });
            break;
        case GLFW_KEY_SPACE:
            if (pickedModel != nullptr) {
                edges_to_collapse = pickedModel->GetMesh()->Q.size();
                pickedModel->simplify(std::ceil(0.1 * edges_to_collapse));
            }
            break;
        }
    }
}